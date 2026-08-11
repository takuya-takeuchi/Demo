import { useEffect } from "react";
import { useLocation, useNavigate } from "react-router-dom";
import { useTranslation } from "react-i18next";
import { registerHankoElements } from "../lib/hanko";
import { useSession } from "../auth/SessionProvider";

interface LocationState {
  from?: string;
}

export default function LoginPage() {
  const navigate = useNavigate();
  const location = useLocation();
  const { status } = useSession();
  const { t, i18n } = useTranslation();

  useEffect(() => {
    registerHankoElements();
  }, []);

  // A successful login is picked up by onSessionCreated in SessionProvider,
  // so all this has to do is watch status and navigate when it changes.
  useEffect(() => {
    if (status === "authenticated") {
      const from = (location.state as LocationState | null)?.from ?? "/todos";
      navigate(from, { replace: true });
    }
  }, [status, navigate, location.state]);

  return (
    <div className="card narrow">
      <h1>{t("login.title")}</h1>
      <p className="muted">{t("login.description")}</p>
      {/* Web component provided by Hanko. The entire login UI fits in this one line. */}
      {/* `lang` is what tells the element which translation set to use. */}
      <hanko-auth lang={i18n.resolvedLanguage} />
    </div>
  );
}
