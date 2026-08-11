import { createContext, useContext, useEffect, useMemo, useState, type ReactNode } from "react";
import { hanko } from "../lib/hanko";

type SessionStatus = "loading" | "authenticated" | "anonymous";

interface SessionContextValue {
  status: SessionStatus;
  userId: string | null;
  logout: () => Promise<void>;
}

const SessionContext = createContext<SessionContextValue | null>(null);

export function SessionProvider({ children }: { children: ReactNode }) {
  const [status, setStatus] = useState<SessionStatus>("loading");
  const [userId, setUserId] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;

    // 初回：既にログイン済みかを Hanko に問い合わせる（リロード対応）
    hanko
      .validateSession()
      .then((res) => {
        if (cancelled) return;
        setStatus(res.is_valid ? "authenticated" : "anonymous");
        setUserId(res.claims?.subject ?? null);
      })
      .catch(() => {
        if (!cancelled) setStatus("anonymous");
      });

    // 以降：ログイン／失効はイベントで受け取る。
    // これらは他のタブでの操作でも発火するので、タブ間で状態が揃う。
    const offCreated = hanko.onSessionCreated((detail) => {
      setStatus("authenticated");
      setUserId(detail.claims?.subject ?? null);
    });

    const offExpired = hanko.onSessionExpired(() => {
      setStatus("anonymous");
      setUserId(null);
    });

    const offLoggedOut = hanko.onUserLoggedOut(() => {
      setStatus("anonymous");
      setUserId(null);
    });

    return () => {
      cancelled = true;
      offCreated();
      offExpired();
      offLoggedOut();
    };
  }, []);

  const value = useMemo<SessionContextValue>(
    () => ({
      status,
      userId,
      logout: async () => {
        await hanko.logout();
        setStatus("anonymous");
        setUserId(null);
      },
    }),
    [status, userId],
  );

  return <SessionContext.Provider value={value}>{children}</SessionContext.Provider>;
}

export function useSession() {
  const ctx = useContext(SessionContext);
  if (!ctx) throw new Error("useSession must be used inside <SessionProvider>");
  return ctx;
}
