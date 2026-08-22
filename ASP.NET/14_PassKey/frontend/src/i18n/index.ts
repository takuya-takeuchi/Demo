import i18n from "i18next";
import LanguageDetector from "i18next-browser-languagedetector";
import { initReactI18next } from "react-i18next";

import { en } from "./locales/en";
import { ja } from "./locales/ja";

export const supportedLanguages = ["en", "ja"] as const;

export type SupportedLanguage = (typeof supportedLanguages)[number];

/**
 * The single i18next instance for the app. Imported for its side effect from main.tsx
 * so that it is initialised before the first render.
 */
void i18n
  .use(LanguageDetector)
  .use(initReactI18next)
  .init({
    resources: {
      en: { translation: en },
      ja: { translation: ja },
    },
    supportedLngs: supportedLanguages,
    // Treat regional variants such as "ja-JP" as "ja" instead of falling back to English.
    nonExplicitSupportedLngs: true,
    fallbackLng: "en",
    // React escapes interpolated values already.
    interpolation: { escapeValue: false },
    detection: {
      // A stored choice wins over the browser setting, so switching languages sticks.
      order: ["localStorage", "navigator"],
      lookupLocalStorage: "lang",
      caches: ["localStorage"],
    },
  });

/**
 * Keep the document language in sync so that screen readers, browser translation prompts
 * and CSS :lang() selectors see the language the user is actually looking at.
 */
function syncDocumentLanguage(language: string) {
  document.documentElement.lang = language;
}

syncDocumentLanguage(i18n.resolvedLanguage ?? "en");
i18n.on("languageChanged", syncDocumentLanguage);

export default i18n;
