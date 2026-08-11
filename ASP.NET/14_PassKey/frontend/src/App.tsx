import { BrowserRouter, Link, Navigate, Route, Routes, useNavigate } from "react-router-dom";
import { SessionProvider, useSession } from "./auth/SessionProvider";
import { ProtectedRoute } from "./auth/ProtectedRoute";
import LoginPage from "./pages/LoginPage";
import TodosPage from "./pages/TodosPage";
import ProfilePage from "./pages/ProfilePage";

function Header() {
  const { status, logout } = useSession();
  const navigate = useNavigate();

  return (
    <header>
      <Link to="/" className="brand">
        Hanko Sample
      </Link>
      <nav>
        {status === "authenticated" ? (
          <>
            <Link to="/todos">Todo</Link>
            <Link to="/profile">アカウント</Link>
            <button
              className="link"
              onClick={async () => {
                await logout();
                navigate("/login", { replace: true });
              }}
            >
              ログアウト
            </button>
          </>
        ) : (
          <Link to="/login">ログイン</Link>
        )}
      </nav>
    </header>
  );
}

export default function App() {
  return (
    <BrowserRouter>
      <SessionProvider>
        <Header />
        <main>
          <Routes>
            <Route path="/login" element={<LoginPage />} />
            <Route element={<ProtectedRoute />}>
              <Route path="/todos" element={<TodosPage />} />
              <Route path="/profile" element={<ProfilePage />} />
            </Route>
            <Route path="*" element={<Navigate to="/todos" replace />} />
          </Routes>
        </main>
      </SessionProvider>
    </BrowserRouter>
  );
}
