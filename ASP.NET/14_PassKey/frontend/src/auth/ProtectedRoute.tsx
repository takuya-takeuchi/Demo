import { Navigate, Outlet, useLocation } from "react-router-dom";
import { useSession } from "./SessionProvider";

/**
 * Route guard that redirects to the login screen when not signed in.
 * This exists for the sake of the user experience, not as a security boundary.
 * The real protection is the JWT verification on the backend.
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
