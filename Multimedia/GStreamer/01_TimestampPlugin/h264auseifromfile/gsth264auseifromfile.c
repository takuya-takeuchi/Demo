#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <gst/video/video.h>
#include <gst/video/video-sei.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>

#define GST_TYPE_H264_AU_SEI_FROM_FILE (gst_h264_au_sei_from_file_get_type())
G_DECLARE_FINAL_TYPE(
    GstH264AuSeiFromFile,
    gst_h264_au_sei_from_file,
    GST,
    H264_AU_SEI_FROM_FILE,
    GstBaseTransform
)

struct _GstH264AuSeiFromFile {
    GstBaseTransform parent;

    gchar *location;
    gchar *uuid_str;
    guint8 uuid[16];
    gboolean uuid_valid;

    FILE *fp;
    gboolean skip_header_done;
    guint64 line_no;

    gboolean repeat_last_on_eof;
    gboolean fail_on_short_csv;

    guint64 last_wallclock_ns;
    gboolean has_last_wallclock;

    GMutex lock;
};

G_DEFINE_TYPE(
    GstH264AuSeiFromFile,
    gst_h264_au_sei_from_file,
    GST_TYPE_BASE_TRANSFORM
)

/* ---------- properties ---------- */

enum {
    PROP_0,
    PROP_LOCATION,
    PROP_UUID,
    PROP_REPEAT_LAST_ON_EOF,
    PROP_FAIL_ON_SHORT_CSV,
    N_PROPERTIES
};

static GParamSpec *props[N_PROPERTIES] = {0};

/* ---------- helpers ---------- */

static gboolean
h264_au_sei_from_file_hexpair_to_byte(const gchar *s, guint8 *out)
{
    guint value = 0;
    if (sscanf(s, "%2x", &value) != 1)
        return FALSE;
    *out = (guint8)value;
    return TRUE;
}

static gboolean
h264_au_sei_from_file_parse_uuid_string_16bytes(const gchar *uuid_str, guint8 out[16])
{
    gchar compact[33];
    gint i = 0, j = 0;

    if (!uuid_str)
        return FALSE;

    for (i = 0; uuid_str[i] != '\0'; i++) {
        if (uuid_str[i] == '-')
            continue;
        if (!g_ascii_isxdigit(uuid_str[i]))
            return FALSE;
        if (j >= 32)
            return FALSE;
        compact[j++] = uuid_str[i];
    }

    if (j != 32)
        return FALSE;

    compact[32] = '\0';

    for (i = 0; i < 16; i++) {
        if (!h264_au_sei_from_file_hexpair_to_byte(&compact[i * 2], &out[i]))
            return FALSE;
    }

    return TRUE;
}

static gboolean
h264_au_sei_from_file_open_if_needed(GstH264AuSeiFromFile *self)
{
    if (self->fp)
        return TRUE;

    if (!self->location || !*self->location) {
        GST_ERROR_OBJECT(self, "location is not set");
        return FALSE;
    }

    self->fp = fopen(self->location, "rb");
    if (!self->fp) {
        GST_ERROR_OBJECT(self, "Failed to open CSV file %s: %s",
                         self->location, g_strerror(errno));
        return FALSE;
    }

    self->skip_header_done = FALSE;
    self->line_no = 0;
    self->has_last_wallclock = FALSE;
    self->last_wallclock_ns = 0;
    return TRUE;
}

static void
h264_au_sei_from_file_close(GstH264AuSeiFromFile *self)
{
    if (self->fp) {
        fclose(self->fp);
        self->fp = NULL;
    }
}

static gboolean
h264_au_sei_from_file_parse_csv_line_for_wallclock(const gchar *line,
                                                   guint64 line_no,
                                                   guint64 *out_wallclock_ns)
{
    gboolean ok = FALSE;
    gchar *copy = NULL;
    gchar *trimmed = NULL;
    gchar **tokens = NULL;

    if (!line)
        return FALSE;

    copy = g_strdup(line);
    trimmed = g_strstrip(copy);

    if (trimmed[0] == '\0')
        goto done;

    /* header */
    if (g_ascii_strncasecmp(trimmed, "sequence,", 9) == 0)
        goto done;

    tokens = g_strsplit(trimmed, ",", -1);

    /* need at least: sequence, pts_ns, dts_ns, wallclock_ns */
    if (!tokens || !tokens[0] || !tokens[1] || !tokens[2] || !tokens[3]) {
        goto done;
    }

    errno = 0;
    gchar *endptr = NULL;
    guint64 wallclock_ns = g_ascii_strtoull(tokens[3], &endptr, 10);
    if (errno != 0 || endptr == tokens[3] || (endptr && *endptr != '\0')) {
        goto done;
    }

    *out_wallclock_ns = wallclock_ns;
    ok = TRUE;

done:
    if (!ok && trimmed && trimmed[0] != '\0') {
        /* header line is not an error */
        if (!(g_ascii_strncasecmp(trimmed, "sequence,", 9) == 0)) {
            g_warning("h264auseifromfile: failed to parse CSV line %" G_GUINT64_FORMAT ": %s",
                      line_no, trimmed);
        }
    }

    g_strfreev(tokens);
    g_free(copy);
    return ok;
}

static gboolean
h264_au_sei_from_file_read_next_wallclock_ns(GstH264AuSeiFromFile *self,
                                             guint64 *out_wallclock_ns,
                                             gboolean *out_from_repeat)
{
    char line[4096];

    *out_from_repeat = FALSE;

    if (!h264_au_sei_from_file_open_if_needed(self))
        return FALSE;

    while (fgets(line, sizeof(line), self->fp) != NULL) {
        self->line_no++;

        if (h264_au_sei_from_file_parse_csv_line_for_wallclock(
                line, self->line_no, out_wallclock_ns)) {
            self->last_wallclock_ns = *out_wallclock_ns;
            self->has_last_wallclock = TRUE;
            return TRUE;
        }

        if (!self->skip_header_done) {
            self->skip_header_done = TRUE;
        }
    }

    if (self->repeat_last_on_eof && self->has_last_wallclock) {
        *out_wallclock_ns = self->last_wallclock_ns;
        *out_from_repeat = TRUE;
        return TRUE;
    }

    return FALSE;
}

/* payload:
 * byte 0      : version = 1
 * byte 1..7   : reserved = 0
 * byte 8..15  : wallclock_ns uint64 big-endian
 */
static gboolean
h264_au_sei_from_file_add_meta(GstH264AuSeiFromFile *self,
                               GstBuffer *buf,
                               guint64 wallclock_ns)
{
    guint8 payload[16] = {0};
    guint64 be_wallclock = GUINT64_TO_BE(wallclock_ns);

    payload[0] = 1;
    memcpy(payload + 8, &be_wallclock, sizeof(be_wallclock));

    GstVideoSEIUserDataUnregisteredMeta* meta = gst_buffer_add_video_sei_user_data_unregistered_meta(
        buf, self->uuid, payload, sizeof(payload));

    return meta != NULL;
}

/* ---------- GstBaseTransform ---------- */

static gboolean
gst_h264_au_sei_from_file_start(GstBaseTransform *base)
{
    GstH264AuSeiFromFile *self = GST_H264_AU_SEI_FROM_FILE(base);

    g_mutex_lock(&self->lock);
    h264_au_sei_from_file_close(self);
    self->skip_header_done = FALSE;
    self->line_no = 0;
    self->has_last_wallclock = FALSE;
    self->last_wallclock_ns = 0;
    g_mutex_unlock(&self->lock);

    return TRUE;
}

static gboolean
gst_h264_au_sei_from_file_stop(GstBaseTransform *base)
{
    GstH264AuSeiFromFile *self = GST_H264_AU_SEI_FROM_FILE(base);

    g_mutex_lock(&self->lock);
    h264_au_sei_from_file_close(self);
    g_mutex_unlock(&self->lock);

    return TRUE;
}

static GstFlowReturn
gst_h264_au_sei_from_file_transform_ip(GstBaseTransform *base, GstBuffer *buf)
{
    GstH264AuSeiFromFile *self = GST_H264_AU_SEI_FROM_FILE(base);
    guint64 wallclock_ns = 0;
    gboolean from_repeat = FALSE;

    g_mutex_lock(&self->lock);

    if (!self->uuid_valid) {
        g_mutex_unlock(&self->lock);
        GST_ERROR_OBJECT(self, "UUID is invalid");
        return GST_FLOW_ERROR;
    }

    if (!h264_au_sei_from_file_read_next_wallclock_ns(self, &wallclock_ns, &from_repeat)) {
        gboolean fail = self->fail_on_short_csv;
        g_mutex_unlock(&self->lock);

        if (fail) {
            GST_ERROR_OBJECT(self, "CSV ended before video AU stream ended");
            return GST_FLOW_ERROR;
        }

        GST_WARNING_OBJECT(self, "CSV ended; passing remaining AU without SEI");
        return GST_FLOW_OK;
    }

    if (!gst_buffer_is_writable(buf)) {
        /* base transform should usually handle this, but keep this safe */
        GST_WARNING_OBJECT(self, "Buffer is not writable; metadata insertion may fail");
    }

    if (!h264_au_sei_from_file_add_meta(self, buf, wallclock_ns)) {
        g_mutex_unlock(&self->lock);
        GST_ERROR_OBJECT(self, "Failed to add GstVideoSEIUserDataUnregisteredMeta");
        return GST_FLOW_ERROR;
    }

    g_mutex_unlock(&self->lock);

    GST_LOG_OBJECT(self,
                   "Added wallclock_ns=%" G_GUINT64_FORMAT "%s",
                   wallclock_ns,
                   from_repeat ? " (reused)" : "");

    return GST_FLOW_OK;
}

/* ---------- GObject ---------- */

static void
gst_h264_au_sei_from_file_set_property(GObject *object,
                                       guint prop_id,
                                       const GValue *value,
                                       GParamSpec *pspec)
{
    GstH264AuSeiFromFile *self = GST_H264_AU_SEI_FROM_FILE(object);

    g_mutex_lock(&self->lock);

    switch (prop_id) {
    case PROP_LOCATION:
        g_free(self->location);
        self->location = g_value_dup_string(value);
        break;
    case PROP_UUID:
    {
        const gchar *s = g_value_get_string(value);
        g_free(self->uuid_str);
        self->uuid_str = g_value_dup_string(value);
        self->uuid_valid = h264_au_sei_from_file_parse_uuid_string_16bytes(s, self->uuid);
        if (!self->uuid_valid) {
            GST_WARNING_OBJECT(self, "Invalid UUID string: %s", s ? s : "(null)");
        }
        break;
    }
    case PROP_REPEAT_LAST_ON_EOF:
        self->repeat_last_on_eof = g_value_get_boolean(value);
        break;
    case PROP_FAIL_ON_SHORT_CSV:
        self->fail_on_short_csv = g_value_get_boolean(value);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }

    g_mutex_unlock(&self->lock);
}

static void
gst_h264_au_sei_from_file_get_property(GObject *object,
                                       guint prop_id,
                                       GValue *value,
                                       GParamSpec *pspec)
{
    GstH264AuSeiFromFile *self = GST_H264_AU_SEI_FROM_FILE(object);

    g_mutex_lock(&self->lock);

    switch (prop_id) {
    case PROP_LOCATION:
        g_value_set_string(value, self->location);
        break;
    case PROP_UUID:
        g_value_set_string(value, self->uuid_str);
        break;
    case PROP_REPEAT_LAST_ON_EOF:
        g_value_set_boolean(value, self->repeat_last_on_eof);
        break;
    case PROP_FAIL_ON_SHORT_CSV:
        g_value_set_boolean(value, self->fail_on_short_csv);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }

    g_mutex_unlock(&self->lock);
}

static void
gst_h264_au_sei_from_file_dispose(GObject *object)
{
    GstH264AuSeiFromFile *self = GST_H264_AU_SEI_FROM_FILE(object);

    g_mutex_lock(&self->lock);
    h264_au_sei_from_file_close(self);
    g_clear_pointer(&self->location, g_free);
    g_clear_pointer(&self->uuid_str, g_free);
    g_mutex_unlock(&self->lock);

    G_OBJECT_CLASS(gst_h264_au_sei_from_file_parent_class)->dispose(object);
}

static void
gst_h264_au_sei_from_file_finalize(GObject *object)
{
    GstH264AuSeiFromFile *self = GST_H264_AU_SEI_FROM_FILE(object);
    g_mutex_clear(&self->lock);
    G_OBJECT_CLASS(gst_h264_au_sei_from_file_parent_class)->finalize(object);
}

static void
gst_h264_au_sei_from_file_class_init(GstH264AuSeiFromFileClass *klass)
{
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    GstBaseTransformClass *trans_class = GST_BASE_TRANSFORM_CLASS(klass);
    GstElementClass *element_class = GST_ELEMENT_CLASS(klass);

    gobject_class->set_property = gst_h264_au_sei_from_file_set_property;
    gobject_class->get_property = gst_h264_au_sei_from_file_get_property;
    gobject_class->dispose = gst_h264_au_sei_from_file_dispose;
    gobject_class->finalize = gst_h264_au_sei_from_file_finalize;

    props[PROP_LOCATION] =
        g_param_spec_string(
            "location",
            "Location",
            "Path to input CSV file",
            "timestamps.csv",
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS);

    props[PROP_UUID] =
        g_param_spec_string(
            "uuid",
            "UUID",
            "UUID for SEI user_data_unregistered payload",
            "086f3693-b7b3-4f2c-9653-21492feee5b8",
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS);

    props[PROP_REPEAT_LAST_ON_EOF] =
        g_param_spec_boolean(
            "repeat-last-on-eof",
            "Repeat last on EOF",
            "Reuse the last wallclock_ns when CSV ends before AU stream ends",
            FALSE,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS);

    props[PROP_FAIL_ON_SHORT_CSV] =
        g_param_spec_boolean(
            "fail-on-short-csv",
            "Fail on short CSV",
            "Return an error when CSV ends before AU stream ends",
            TRUE,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS);

    g_object_class_install_properties(gobject_class, N_PROPERTIES, props);

    gst_element_class_set_static_metadata(
        element_class,
        "H264 AU SEI From File",
        "Codec/Video",
        "Reads wallclock_ns from CSV and attaches SEI user-data metadata per H.264 AU",
        "OpenAI");

    {
        GstCaps *caps = gst_caps_from_string("video/x-h264, alignment=(string)au");
        gst_element_class_add_pad_template(
            element_class,
            gst_pad_template_new("sink", GST_PAD_SINK, GST_PAD_ALWAYS, caps));
        gst_element_class_add_pad_template(
            element_class,
            gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS, caps));
        gst_caps_unref(caps);
    }

    trans_class->start = gst_h264_au_sei_from_file_start;
    trans_class->stop = gst_h264_au_sei_from_file_stop;
    trans_class->transform_ip = gst_h264_au_sei_from_file_transform_ip;
}

static void
gst_h264_au_sei_from_file_init(GstH264AuSeiFromFile *self)
{
    self->location = g_strdup("timestamps.csv");
    self->uuid_str = g_strdup("086f3693-b7b3-4f2c-9653-21492feee5b8");
    self->uuid_valid =
        h264_au_sei_from_file_parse_uuid_string_16bytes(self->uuid_str, self->uuid);

    self->fp = NULL;
    self->skip_header_done = FALSE;
    self->line_no = 0;
    self->repeat_last_on_eof = FALSE;
    self->fail_on_short_csv = TRUE;
    self->last_wallclock_ns = 0;
    self->has_last_wallclock = FALSE;
    g_mutex_init(&self->lock);

    gst_base_transform_set_in_place(GST_BASE_TRANSFORM(self), TRUE);
    gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(self), FALSE);
}

/* ---------- plugin entry ---------- */

static gboolean
plugin_init(GstPlugin *plugin)
{
    return gst_element_register(
        plugin,
        "h264auseifromfile",
        GST_RANK_NONE,
        GST_TYPE_H264_AU_SEI_FROM_FILE);
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    h264auseifromfile,
    "Reads wallclock_ns from CSV and adds SEI metadata per H.264 AU",
    plugin_init,
    "1.0.0",
    "MIT",
    "h264auseifromfile",
    "https://taktak.jp"
)
