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
      <h1>ログイン / 新規登録</h1>
      <p className="muted">
        パスキー（生体認証・端末 PIN）またはメールに届くコードでログインできます。
      </p>
      {/* Web component provided by Hanko. The entire login UI fits in this one line. */}
      <hanko-auth />
    </div>
  );
}
