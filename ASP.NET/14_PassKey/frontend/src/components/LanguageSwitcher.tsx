import { useTranslation } from "react-i18next";

import { supportedLanguages } from "../i18n";

/**
 * Language picker for the header. Changing the language stores the choice
 * (see the detection settings in i18n/index.ts), so it survives a reload.
 */
export function LanguageSwitcher() {
  const { t, i18n } = useTranslation();

  return (
    <select
      className="lang"
      aria-label={t("language.label")}
      value={i18n.resolvedLanguage}
      onChange={(e) => void i18n.changeLanguage(e.target.value)}
    >
      {supportedLanguages.map((language) => (
        <option key={language} value={language}>
          {t(`language.${language}`)}
        </option>
      ))}
    </select>
  );
}
