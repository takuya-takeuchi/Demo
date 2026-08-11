import { useCallback, useEffect, useState } from "react";
import { api, ApiError, type Me, type Todo } from "../lib/api";

export default function TodosPage() {
  const [me, setMe] = useState<Me | null>(null);
  const [todos, setTodos] = useState<Todo[]>([]);
  const [title, setTitle] = useState("");
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const load = useCallback(async () => {
    try {
      setError(null);
      const [meRes, todosRes] = await Promise.all([api.me(), api.listTodos()]);
      setMe(meRes);
      setTodos(todosRes);
    } catch (e) {
      setError(e instanceof ApiError ? e.message : "読み込みに失敗しました。");
    }
  }, []);

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
      setError(e instanceof ApiError ? e.message : "追加に失敗しました。");
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
      <h1>Todo</h1>
      {me && (
        <p className="muted">
          {me.email} としてログイン中（Hanko user_id: <code>{me.id}</code>）
        </p>
      )}

      {error && <p className="error">{error}</p>}

      <form className="row" onSubmit={add}>
        <input
          value={title}
          onChange={(e) => setTitle(e.target.value)}
          placeholder="やることを入力"
          maxLength={500}
        />
        <button type="submit" disabled={busy || !title.trim()}>
          追加
        </button>
      </form>

      {todos.length === 0 ? (
        <p className="muted">まだ何もありません。</p>
      ) : (
        <ul className="todos">
          {todos.map((todo) => (
            <li key={todo.id} className={todo.isDone ? "done" : undefined}>
              <label>
                <input type="checkbox" checked={todo.isDone} onChange={() => void toggle(todo)} />
                <span>{todo.title}</span>
              </label>
              <button className="link" onClick={() => void remove(todo)} aria-label="削除">
                削除
              </button>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
