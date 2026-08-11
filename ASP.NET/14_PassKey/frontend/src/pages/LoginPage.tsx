import { useEffect } from "react";
import { useLocation, useNavigate } from "react-router-dom";
import { registerHankoElements } from "../lib/hanko";
import { useSession } from "../auth/SessionProvider";

interface LocationState {
  from?: string;
}

export default function LoginPage() {
  const navigate = useNavigate();
  const location = useLocation();
  const { status } = useSession();

  useEffect(() => {
    registerHankoElements();
  }, []);

  // ログイン成功は SessionProvider の onSessionCreated が拾うので、
  // ここでは status の変化を見て遷移するだけでよい。
  useEffect(() => {
    if (status === "authenticated") {
      const from = (location.state as LocationState | null)?.from ?? "/todos";
      navigate(from, { replace: true });
    }
  }, [status, navigate, location.state]);

  return (
    <div className="card narrow">
      <h1>ログイン / 新規登録</h1>
      <p className="muted">
        パスキー（生体認証・端末 PIN）またはメールに届くコードでログインできます。
      </p>
      {/* Hanko が提供する Web Component。ログイン UI 一式がこの 1 行に入っている */}
      <hanko-auth />
    </div>
  );
}
