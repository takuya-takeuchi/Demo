#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <gst/video/video.h>

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <errno.h>

#define GST_TYPE_H264_AU_TIMESTAMP_DUMP (gst_h264_au_timestamp_dump_get_type())
G_DECLARE_FINAL_TYPE(
    GstH264AuTimestampDump,
    gst_h264_au_timestamp_dump,
    GST,
    H264_AU_TIMESTAMP_DUMP,
    GstBaseTransform
)

struct _GstH264AuTimestampDump {
    GstBaseTransform parent;

    gchar *location;
    FILE *fp;
    gboolean header_written;
    gboolean append;
    gboolean flush_each_line;

    guint32 sequence;
    GMutex lock;
};

G_DEFINE_TYPE(
    GstH264AuTimestampDump,
    gst_h264_au_timestamp_dump,
    GST_TYPE_BASE_TRANSFORM
)

/* ---------- properties ---------- */

enum {
    PROP_0,
    PROP_LOCATION,
    PROP_APPEND,
    PROP_FLUSH_EACH_LINE,
    N_PROPERTIES
};

static GParamSpec *props[N_PROPERTIES] = {0};

/* ---------- helpers ---------- */

static guint64
h264_au_timestamp_dump_now_wallclock_ns(void)
{
    return ((guint64) g_get_real_time()) * 1000ULL;
}

static guint64
h264_au_timestamp_dump_now_monotonic_ns(void)
{
    return ((guint64) g_get_monotonic_time()) * 1000ULL;
}

static guint64
h264_au_timestamp_dump_pts_or_invalid(GstBuffer *buf)
{
    if (GST_BUFFER_PTS_IS_VALID(buf))
        return (guint64) GST_BUFFER_PTS(buf);
    return G_MAXUINT64;
}

static guint64
h264_au_timestamp_dump_dts_or_invalid(GstBuffer *buf)
{
    if (GST_BUFFER_DTS_IS_VALID(buf))
        return (guint64) GST_BUFFER_DTS(buf);
    return G_MAXUINT64;
}

static gboolean
h264_au_timestamp_dump_ensure_parent_dir(const gchar *path)
{
    gboolean ok = TRUE;
    gchar *dir = NULL;

    if (!path || !*path)
        return FALSE;

    dir = g_path_get_dirname(path);
    if (!dir)
        return FALSE;

    if (g_strcmp0(dir, ".") != 0) {
        if (g_mkdir_with_parents(dir, 0755) != 0) {
            ok = FALSE;
        }
    }

    g_free(dir);
    return ok;
}

static void
h264_au_timestamp_dump_close_csv(GstH264AuTimestampDump *self)
{
    if (self->fp) {
        fflush(self->fp);
        fclose(self->fp);
        self->fp = NULL;
    }
}

static gboolean
h264_au_timestamp_dump_open_csv_if_needed(GstH264AuTimestampDump *self)
{
    const char *mode;

    if (self->fp)
        return TRUE;

    if (!self->location || !*self->location) {
        GST_ERROR_OBJECT(self, "location is not set");
        return FALSE;
    }

    if (!h264_au_timestamp_dump_ensure_parent_dir(self->location)) {
        GST_ERROR_OBJECT(self, "Failed to create parent directory for %s", self->location);
        return FALSE;
    }

    mode = self->append ? "a" : "w";
    self->fp = fopen(self->location, mode);
    if (!self->fp) {
        GST_ERROR_OBJECT(self, "Failed to open CSV file %s: %s",
                         self->location, g_strerror(errno));
        return FALSE;
    }

    if (!self->append) {
        self->header_written = FALSE;
    } else {
        long pos = 0;
        if (fseek(self->fp, 0, SEEK_END) == 0) {
            pos = ftell(self->fp);
            if (pos > 0)
                self->header_written = TRUE;
        }
    }

    if (!self->header_written) {
        fprintf(self->fp, "sequence,pts_ns,dts_ns,wallclock_ns,monotonic_ns\n");
        self->header_written = TRUE;
        if (self->flush_each_line)
            fflush(self->fp);
    }

    return TRUE;
}

/* ---------- GstBaseTransform vfunc ---------- */

static gboolean
gst_h264_au_timestamp_dump_start(GstBaseTransform *base)
{
    GstH264AuTimestampDump *self = GST_H264_AU_TIMESTAMP_DUMP(base);

    g_mutex_lock(&self->lock);
    self->sequence = 0;
    self->header_written = FALSE;
    h264_au_timestamp_dump_close_csv(self);
    g_mutex_unlock(&self->lock);

    return TRUE;
}

static gboolean
gst_h264_au_timestamp_dump_stop(GstBaseTransform *base)
{
    GstH264AuTimestampDump *self = GST_H264_AU_TIMESTAMP_DUMP(base);

    g_mutex_lock(&self->lock);
    h264_au_timestamp_dump_close_csv(self);
    g_mutex_unlock(&self->lock);

    return TRUE;
}

static GstFlowReturn
gst_h264_au_timestamp_dump_transform_ip(GstBaseTransform *base, GstBuffer *buf)
{
    GstH264AuTimestampDump *self = GST_H264_AU_TIMESTAMP_DUMP(base);

    guint32 seq;
    guint64 pts_ns;
    guint64 dts_ns;
    guint64 wallclock_ns;
    guint64 monotonic_ns;

    g_mutex_lock(&self->lock);

    if (!h264_au_timestamp_dump_open_csv_if_needed(self)) {
        g_mutex_unlock(&self->lock);
        return GST_FLOW_ERROR;
    }

    seq = self->sequence++;
    pts_ns = h264_au_timestamp_dump_pts_or_invalid(buf);
    dts_ns = h264_au_timestamp_dump_dts_or_invalid(buf);
    wallclock_ns = h264_au_timestamp_dump_now_wallclock_ns();
    monotonic_ns = h264_au_timestamp_dump_now_monotonic_ns();

    fprintf(self->fp,
            "%" PRIu32 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 "\n",
            seq, pts_ns, dts_ns, wallclock_ns, monotonic_ns);

    if (self->flush_each_line)
        fflush(self->fp);

    g_mutex_unlock(&self->lock);

    GST_LOG_OBJECT(self,
                   "CSV: seq=%" PRIu32 " pts=%" PRIu64 " dts=%" PRIu64
                   " wall=%" PRIu64 " mono=%" PRIu64,
                   seq, pts_ns, dts_ns, wallclock_ns, monotonic_ns);

    return GST_FLOW_OK;
}

/* ---------- state / event ---------- */

static gboolean
gst_h264_au_timestamp_dump_sink_event(GstBaseTransform *base, GstEvent *event)
{
    GstH264AuTimestampDump *self = GST_H264_AU_TIMESTAMP_DUMP(base);

    switch (GST_EVENT_TYPE(event)) {
    case GST_EVENT_EOS:
        g_mutex_lock(&self->lock);
        if (self->fp)
            fflush(self->fp);
        g_mutex_unlock(&self->lock);
        break;
    default:
        break;
    }

    return GST_BASE_TRANSFORM_CLASS(gst_h264_au_timestamp_dump_parent_class)
        ->sink_event(base, event);
}

/* ---------- GObject ---------- */

static void
gst_h264_au_timestamp_dump_set_property(GObject *object,
                                        guint prop_id,
                                        const GValue *value,
                                        GParamSpec *pspec)
{
    GstH264AuTimestampDump *self = GST_H264_AU_TIMESTAMP_DUMP(object);

    g_mutex_lock(&self->lock);

    switch (prop_id) {
    case PROP_LOCATION:
        g_free(self->location);
        self->location = g_value_dup_string(value);
        break;
    case PROP_APPEND:
        self->append = g_value_get_boolean(value);
        break;
    case PROP_FLUSH_EACH_LINE:
        self->flush_each_line = g_value_get_boolean(value);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }

    g_mutex_unlock(&self->lock);
}

static void
gst_h264_au_timestamp_dump_get_property(GObject *object,
                                        guint prop_id,
                                        GValue *value,
                                        GParamSpec *pspec)
{
    GstH264AuTimestampDump *self = GST_H264_AU_TIMESTAMP_DUMP(object);

    g_mutex_lock(&self->lock);

    switch (prop_id) {
    case PROP_LOCATION:
        g_value_set_string(value, self->location);
        break;
    case PROP_APPEND:
        g_value_set_boolean(value, self->append);
        break;
    case PROP_FLUSH_EACH_LINE:
        g_value_set_boolean(value, self->flush_each_line);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }

    g_mutex_unlock(&self->lock);
}

static void
gst_h264_au_timestamp_dump_dispose(GObject *object)
{
    GstH264AuTimestampDump *self = GST_H264_AU_TIMESTAMP_DUMP(object);

    g_mutex_lock(&self->lock);
    h264_au_timestamp_dump_close_csv(self);
    g_clear_pointer(&self->location, g_free);
    g_mutex_unlock(&self->lock);

    G_OBJECT_CLASS(gst_h264_au_timestamp_dump_parent_class)->dispose(object);
}

static void
gst_h264_au_timestamp_dump_finalize(GObject *object)
{
    GstH264AuTimestampDump *self = GST_H264_AU_TIMESTAMP_DUMP(object);
    g_mutex_clear(&self->lock);
    G_OBJECT_CLASS(gst_h264_au_timestamp_dump_parent_class)->finalize(object);
}

static void
gst_h264_au_timestamp_dump_class_init(GstH264AuTimestampDumpClass *klass)
{
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    GstBaseTransformClass *trans_class = GST_BASE_TRANSFORM_CLASS(klass);
    GstElementClass *element_class = GST_ELEMENT_CLASS(klass);

    gobject_class->set_property = gst_h264_au_timestamp_dump_set_property;
    gobject_class->get_property = gst_h264_au_timestamp_dump_get_property;
    gobject_class->dispose = gst_h264_au_timestamp_dump_dispose;
    gobject_class->finalize = gst_h264_au_timestamp_dump_finalize;

    props[PROP_LOCATION] =
        g_param_spec_string(
            "location",
            "Location",
            "Path to output CSV file",
            "timestamps.csv",
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS);

    props[PROP_APPEND] =
        g_param_spec_boolean(
            "append",
            "Append",
            "Append to an existing CSV file instead of overwriting",
            FALSE,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS);

    props[PROP_FLUSH_EACH_LINE] =
        g_param_spec_boolean(
            "flush-each-line",
            "Flush each line",
            "Flush file after each CSV line",
            TRUE,
            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS);

    g_object_class_install_properties(gobject_class, N_PROPERTIES, props);

    gst_element_class_set_static_metadata(
        element_class,
        "H264 AU Timestamp Dump",
        "Codec/Video",
        "Writes one CSV row per H.264 access unit with pass-through timestamps",
        "OpenAI");

    GstCaps *caps = gst_caps_from_string("video/x-h264, alignment=(string)au");
    gst_element_class_add_pad_template(
        element_class,
        gst_pad_template_new("sink", GST_PAD_SINK, GST_PAD_ALWAYS, caps));
    gst_element_class_add_pad_template(
        element_class,
        gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS, caps));
    gst_caps_unref(caps);

    trans_class->start = gst_h264_au_timestamp_dump_start;
    trans_class->stop = gst_h264_au_timestamp_dump_stop;
    trans_class->transform_ip = gst_h264_au_timestamp_dump_transform_ip;
    trans_class->sink_event = gst_h264_au_timestamp_dump_sink_event;
}

static void
gst_h264_au_timestamp_dump_init(GstH264AuTimestampDump *self)
{
    self->location = g_strdup("timestamps.csv");
    self->fp = NULL;
    self->header_written = FALSE;
    self->append = FALSE;
    self->flush_each_line = TRUE;
    self->sequence = 0;
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
        "h264autimestampdump",
        GST_RANK_NONE,
        GST_TYPE_H264_AU_TIMESTAMP_DUMP);
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    h264autimestampdump,
    "Writes per-H.264-AU timestamps to CSV",
    plugin_init,
    "1.0.0",
    "MIT",
    "h264autimestampdump",
    "https://taktak.jp"
)