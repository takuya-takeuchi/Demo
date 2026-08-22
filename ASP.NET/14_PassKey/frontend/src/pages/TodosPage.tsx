import { useCallback, useEffect, useState } from "react";
import { Trans, useTranslation } from "react-i18next";
import { api, ApiError, type Me, type Todo } from "../lib/api";

export default function TodosPage() {
  const { t } = useTranslation();
  const [me, setMe] = useState<Me | null>(null);
  const [todos, setTodos] = useState<Todo[]>([]);
  const [title, setTitle] = useState("");
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  // Errors from the API arrive as a translation key, so they are rendered in the
  // language that is active when they are caught.
  const describe = useCallback(
    (e: unknown, fallback: string) =>
      e instanceof ApiError ? t(e.messageKey, e.messageValues) : fallback,
    [t],
  );

  const load = useCallback(async () => {
    try {
      setError(null);
      const [meRes, todosRes] = await Promise.all([api.me(), api.listTodos()]);
      setMe(meRes);
      setTodos(todosRes);
    } catch (e) {
      setError(describe(e, t("errors.load")));
    }
  }, [describe, t]);

  useEffect(() => {
    void load();
  }, [load]);

  async function add(e: React.FormEvent) {
    e.preventDefault();
    if (!title.trim() || busy) return;

    setBusy(true);
    try {
      const created = await api.createTodo(title.trim());
      setTodos((prev) => [created, ...prev]);
      setTitle("");
    } catch (e) {
      setError(describe(e, t("errors.add")));
    } finally {
      setBusy(false);
    }
  }

  async function toggle(todo: Todo) {
    const updated = await api.updateTodo(todo.id, { isDone: !todo.isDone });
    setTodos((prev) => prev.map((t) => (t.id === updated.id ? updated : t)));
  }

  async function remove(todo: Todo) {
    await api.deleteTodo(todo.id);
    setTodos((prev) => prev.filter((t) => t.id !== todo.id));
  }

  return (
    <div className="card">
      <h1>{t("todos.title")}</h1>
      {me && (
        <p className="muted">
          {/* <Trans> keeps the <code> markup in the component and out of the translations. */}
          <Trans
            i18nKey="todos.signedInAs"
            values={{ email: me.email, userId: me.id }}
            components={{ code: <code /> }}
          />
        </p>
      )}

      {error && <p className="error">{error}</p>}

      <form className="row" onSubmit={add}>
        <input
          value={title}
          onChange={(e) => setTitle(e.target.value)}
          placeholder={t("todos.placeholder")}
          maxLength={500}
        />
        <button type="submit" disabled={busy || !title.trim()}>
          {t("todos.add")}
        </button>
      </form>

      {todos.length === 0 ? (
        <p className="muted">{t("todos.empty")}</p>
      ) : (
        <ul className="todos">
          {todos.map((todo) => (
            <li key={todo.id} className={todo.isDone ? "done" : undefined}>
              <label>
                <input type="checkbox" checked={todo.isDone} onChange={() => void toggle(todo)} />
                <span>{todo.title}</span>
              </label>
              <button
                className="link"
                onClick={() => void remove(todo)}
                aria-label={t("todos.delete")}
              >
                {t("todos.delete")}
              </button>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
