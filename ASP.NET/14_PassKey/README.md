# Hanko + ASP.NET Core + React + SQLite サンプル

パスキー認証を [Hanko](https://github.com/teamhanko/hanko)（セルフホスト）に任せ、
アプリのデータは SQLite に持つ最小構成の Web アプリです。ログイン後に自分専用の Todo を
作成・更新・削除できます。

```
React (Vite, :5173)            ─ hanko-elements でログイン UI
   │  ① ログイン                    ↘ hanko Cookie / Bearer トークン
   ▼
Hanko (Go, :8000) ── Postgres      ② JWKS を公開
   │
   ▼
ASP.NET Core API (:5080) ── SQLite (app.db)
        ③ JWKS で JWT を検証し、sub をキーにアプリ側ユーザーへ紐付け
```

**Hanko 自身は SQLite に対応していません（PostgreSQL / MySQL のみ）。**
そのため認証基盤用に Postgres を 1 つ立て、アプリのデータだけを SQLite に置いています。

---

## 必要なもの

| ツール | バージョン |
|---|---|
| Docker / Docker Compose | 任意の新しめのもの |
| .NET SDK | 10.0 |
| Node.js | 20 以上（22 で確認） |

---

## 起動手順

### 1. Hanko を起動する

```bash
docker compose up -d
```

| URL | 用途 |
|---|---|
| http://localhost:8000 | Hanko Public API（フロント・バックエンドが使う） |
| http://localhost:8001 | Hanko Admin API（**外部に公開しないこと**） |
| http://localhost:8080 | mailslurper。確認コードのメールはここで読む |

初回は `hanko-migrate` がスキーマを作ってから `hanko` が起動します。

```bash
docker compose logs -f hanko          # 起動確認
curl http://localhost:8000/.well-known/jwks.json   # 公開鍵が返れば OK
```

### 2. バックエンド（ASP.NET Core）

```bash
cd backend
dotnet run --project HankoApp.Api
# → http://localhost:5080
```

`app.db`（SQLite）は初回起動時に自動生成されます。

### 3. フロントエンド（React）

```bash
cd frontend
cp .env.example .env
npm install
npm run dev
# → http://localhost:5173
```

ブラウザで http://localhost:5173 を開き、メールアドレスを入力 →
mailslurper (http://localhost:8080) に届いたコードを入力 → パスキーを登録、で完了です。

> パスキーは `localhost` に紐付きます。`127.0.0.1` で開くと別オリジン扱いになり
> 登録済みのパスキーが使えないので、必ず `localhost` でアクセスしてください。

---

## ディレクトリ構成

```
.
├─ docker-compose.yml          Hanko + Postgres + mailslurper
├─ hanko/config.yaml           Hanko の設定（パスキー・セッション・CORS）
├─ backend/
│  └─ HankoApp.Api/
│     ├─ Program.cs                    DI・ミドルウェア・起動処理
│     ├─ Auth/
│     │  ├─ HankoOptions.cs            appsettings の Hanko セクション
│     │  ├─ HankoJwksRetriever.cs      JWKS を読んで署名鍵にする
│     │  ├─ HankoAuthenticationExtensions.cs  JwtBearer の設定本体
│     │  ├─ HankoSessionValidator.cs   /sessions/validate を叩く（任意）
│     │  └─ CurrentUserService.cs      初回アクセス時にユーザー行を作る
│     ├─ Data/AppDbContext.cs
│     ├─ Models/{AppUser,TodoItem}.cs
│     └─ Endpoints/TodoEndpoints.cs    /api/me, /api/todos
└─ frontend/
   └─ src/
      ├─ lib/hanko.ts          SDK インスタンスと Web Components の登録
      ├─ lib/api.ts            バックエンド呼び出し（Cookie + Bearer 両対応）
      ├─ auth/SessionProvider.tsx  ログイン状態をアプリ全体で共有
      ├─ auth/ProtectedRoute.tsx   未ログインなら /login へ
      └─ pages/{LoginPage,TodosPage,ProfilePage}.tsx
```

---

## 認証の流れ

1. `<hanko-auth>` がログイン UI を描画し、Hanko と直接やり取りする。
2. ログイン成功時、Hanko が `hanko` という名前の Cookie にセッショントークン（JWS 形式の JWT）を入れる。
3. React は `hanko.onSessionCreated` / `onSessionExpired` でログイン状態を受け取る。
   これらは**他のタブの操作でも発火する**ので、タブ間で状態が揃う。
4. API 呼び出しでは Cookie が自動で送られる（`credentials: "include"`）。
   保険として `Authorization: Bearer` も付けている。
5. ASP.NET Core は Hanko の `/.well-known/jwks.json` から公開鍵を取得して署名を検証し、
   `sub` クレーム（Hanko の user_id）で SQLite のユーザー行を引く。

### なぜ JWKS を直接読んでいるか

Hanko は OIDC ディスカバリ文書（`/.well-known/openid-configuration`）を公開していないため、
`JwtBearerOptions.Authority` を指定する通常のやり方が使えません。
`HankoJwksRetriever` で JWKS だけを読み、`ConfigurationManager` に渡しています。
鍵のキャッシュとローテーション追従は `ConfigurationManager` がやってくれます。

---

## 設定

`backend/HankoApp.Api/appsettings.json`：

| キー | 意味 |
|---|---|
| `Hanko:ApiUrl` | ブラウザから見た Hanko の URL |
| `Hanko:InternalApiUrl` | サーバーから見た Hanko の URL。Docker 内なら `http://hanko:8000` |
| `Hanko:CookieName` | セッション Cookie 名（既定 `hanko`） |
| `Hanko:ValidateAudience` | `aud` を検証するか。Hanko Cloud なら true 推奨 |
| `Hanko:ValidateSessionRemotely` | true にすると毎回 `/sessions/validate` も叩く（後述） |
| `Cors:AllowedOrigins` | フロントのオリジン。`AllowCredentials` のためワイルドカード不可 |

環境変数で上書きする場合は `Hanko__InternalApiUrl=...` のように `__` 区切りにします。

---

## 実運用に移すときの注意

- **`hanko/config.yaml` の `secrets.keys` を必ず差し替える。** `openssl rand -hex 32` などで生成する。
- **`session.cookie.secure` を `true` にする。** HTTPS 前提。`webauthn.relying_party.id` も本番ドメインに変更する
  （変更すると既存パスキーは無効になる）。
- **Admin API (:8001) を外部に出さない。** ユーザーの作成・削除ができてしまう。
- **メール送信を本物の SMTP に向ける。** mailslurper は開発専用。
- **スキーマ管理を EF Core Migrations に切り替える。**
  現状は起動時 `EnsureCreated()` なので、モデルを変更しても既存 DB に反映されない。

  ```bash
  dotnet tool install --global dotnet-ef
  dotnet ef migrations add Initial --project backend/HankoApp.Api
  ```

- **ログアウトを即時反映したいなら `Hanko:ValidateSessionRemotely` を `true` に。**
  JWT の署名検証だけだと、ログアウト後も `exp`（既定 12 時間）まではトークンが通ってしまう。
  true にすると毎リクエストで Hanko に問い合わせるので、重要な操作だけ
  `HankoSessionValidator` を直接呼ぶ運用でもよい。
- **SQLite の同時書き込み。** 起動時に WAL を有効にしているが、書き込みが多いなら Postgres への移行を検討する。

---

## 動作確認済みの内容

バックエンド API（`/api/me`, `/api/todos`）とブラウザ操作の両方を自動テストで確認しています。

- Cookie / `Authorization: Bearer` のどちらでも認証が通る
- トークン無し・期限切れ・署名改ざんはいずれも 401
- 他人の Todo は一覧に出ず、更新・削除も 404（所有者チェックを WHERE に入れている）
- ログイン直後に API が同時に走っても、ユーザー行の初回作成が競合しない
- 許可していないオリジンには CORS ヘッダを返さない

---

## つまずきやすい点

| 症状 | 原因と対処 |
|---|---|
| API がすべて 401 | Hanko が起動していない／`Hanko:InternalApiUrl` が違う。`curl http://localhost:8000/.well-known/jwks.json` で確認 |
| ブラウザで CORS エラー | `Cors:AllowedOrigins`（backend）と `server.public.cors.allow_origins`（hanko/config.yaml）の両方にフロントの URL が必要 |
| Cookie が API に送られない | フロントと API のドメインが違う。`localhost` 同士ならポートが違っても送られる。別ドメインなら Bearer 方式を使う |
| パスキーが登録できない | `webauthn.relying_party.origins` にフロントの URL が入っているか確認。HTTPS か localhost でないと WebAuthn は動かない |
| `SQLite does not support ... DateTimeOffset in ORDER BY` | エンティティの日時は `DateTime`（UTC）で持つ。本サンプルはそうしている |

---

## 参考

- [teamhanko/hanko](https://github.com/teamhanko/hanko)
- [Hanko ドキュメント](https://docs.hanko.io)
- [セッションと JWT の仕様](https://docs.hanko.io/guides/sessions)
