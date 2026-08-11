import type { en } from "./en";

/**
 * Japanese resources. Typed against the English resources so that the two
 * always carry exactly the same keys.
 */
export const ja: typeof en = {
  app: {
    name: "Hanko サンプル",
  },
  nav: {
    todos: "Todo",
    account: "アカウント",
    login: "ログイン",
    logout: "ログアウト",
  },
  language: {
    label: "言語",
    en: "English",
    ja: "日本語",
  },
  session: {
    checking: "セッションを確認中…",
  },
  login: {
    title: "ログイン / 新規登録",
    description: "パスキー（生体認証・端末 PIN）またはメールに届くコードでログインできます。",
  },
  todos: {
    title: "Todo",
    signedInAs: "{{email}} としてログイン中（Hanko user_id: <code>{{userId}}</code>）",
    placeholder: "やることを入力",
    add: "追加",
    empty: "まだ何もありません。",
    delete: "削除",
  },
  profile: {
    title: "アカウント設定",
    description: "パスキーの追加・削除、メールアドレスの変更などは Hanko 側の画面で行います。",
  },
  errors: {
    sessionExpired: "セッションが無効です。ログインし直してください。",
    request: "リクエストに失敗しました（{{status}} {{statusText}}）。",
    load: "読み込みに失敗しました。",
    add: "追加に失敗しました。",
  },
};
