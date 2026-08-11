/**
 * English resources. This file is the source of truth for the key structure:
 * every other locale is typed as `typeof en`, so a missing or misspelled key
 * is a compile error rather than a blank label at runtime.
 */
export const en = {
  app: {
    name: "Hanko Sample",
  },
  nav: {
    todos: "Todo",
    account: "Account",
    login: "Sign in",
    logout: "Sign out",
  },
  language: {
    label: "Language",
    en: "English",
    ja: "日本語",
  },
  session: {
    checking: "Checking your session…",
  },
  login: {
    title: "Sign in / Sign up",
    description:
      "Sign in with a passkey (biometrics or a device PIN), or with a code sent to your email address.",
  },
  todos: {
    title: "Todo",
    // <code> is supplied by the caller via <Trans>, so the markup stays out of the translation.
    signedInAs: "Signed in as {{email}} (Hanko user_id: <code>{{userId}}</code>)",
    placeholder: "What needs doing?",
    add: "Add",
    empty: "Nothing here yet.",
    delete: "Delete",
  },
  profile: {
    title: "Account settings",
    description:
      "Adding or removing passkeys and changing your email address are handled by Hanko's own screens.",
  },
  errors: {
    sessionExpired: "Your session is no longer valid. Please sign in again.",
    request: "The request failed ({{status}} {{statusText}}).",
    load: "Failed to load.",
    add: "Failed to add.",
  },
};
