import "i18next";

import type { en } from "./locales/en";

/**
 * Teaches TypeScript the shape of the resources, so t("todos.titel") is a compile error
 * instead of a string that silently renders as its own key.
 */
declare module "i18next" {
  interface CustomTypeOptions {
    defaultNS: "translation";
    resources: {
      translation: typeof en;
    };
  }
}
