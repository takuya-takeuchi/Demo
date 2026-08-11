import { useEffect } from "react";
import { registerHankoElements } from "../lib/hanko";

export default function ProfilePage() {
  useEffect(() => {
    registerHankoElements();
  }, []);

  return (
    <div className="card narrow">
      <h1>アカウント設定</h1>
      <p className="muted">
        パスキーの追加・削除、メールアドレスの変更などは Hanko 側の画面で行います。
      </p>
      <hanko-profile />
    </div>
  );
}
