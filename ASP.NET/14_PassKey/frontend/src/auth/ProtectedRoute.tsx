import { Navigate, Outlet, useLocation } from "react-router-dom";
import { useSession } from "./SessionProvider";

/**
 * 未ログインならログイン画面へ飛ばすルートガード。
 * ここは体験のためのガードであって、セキュリティ境界ではない。
 * 実際の保護はバックエンド側の JWT 検証で行われている。
 */
export function ProtectedRoute() {
  const { status } = useSession();
  const location = useLocation();

  if (status === "loading") {
    return <p className="muted">セッションを確認中…</p>;
  }

  if (status === "anonymous") {
    return <Navigate to="/login" replace state={{ from: location.pathname }} />;
  }

  return <Outlet />;
}
