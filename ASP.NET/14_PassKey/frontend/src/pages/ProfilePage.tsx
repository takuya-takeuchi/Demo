import { useEffect } from "react";
import { useTranslation } from "react-i18next";
import { registerHankoElements } from "../lib/hanko";

export default function ProfilePage() {
  const { t, i18n } = useTranslation();

  useEffect(() => {
    registerHankoElements();
  }, []);

  return (
    <div className="card narrow">
      <h1>{t("profile.title")}</h1>
      <p className="muted">{t("profile.description")}</p>
      {/* `lang` is what tells the Hanko element which translation set to use. */}
      <hanko-profile lang={i18n.resolvedLanguage} />
    </div>
  );
}
