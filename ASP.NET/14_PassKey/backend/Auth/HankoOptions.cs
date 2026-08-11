namespace Demo.Auth
{

    public class HankoOptions
    {

        #region Properties

        public const string SectionName = "Hanko";

        /// <summary>
        /// このサーバーから Hanko に到達するための URL。JWKS の取得とセッション検証に使う。
        /// ブラウザはここを直接叩かず、必ず /auth 経由（YARP のプロキシ）で通るため、
        /// Hanko は内部ネットワークにだけ公開されていればよい。
        /// docker compose 内で動かす場合はサービス名（http://hanko:8000）を指定する。
        /// </summary>
        public string InternalApiUrl { get; set; } = "http://localhost:8000";

        /// <summary>セッショントークンを載せる Cookie 名。Hanko の既定値は "hanko"。</summary>
        public string CookieName { get; set; } = "hanko";

        /// <summary>
        /// aud クレームを検証するか。Hanko Cloud では App URL が aud に入るので true 推奨。
        /// セルフホストでは config.yaml の session.audience を設定していない限り false のままにする。
        /// </summary>
        public bool ValidateAudience { get; set; }

        public string[] ValidAudiences { get; set; } = [];

        /// <summary>
        /// true にすると、JWT 署名検証に加えて Hanko の /sessions/validate を呼び、
        /// サーバー側でセッションが失効していないかを確認する（ログアウトの即時反映用）。
        /// </summary>
        public bool ValidateSessionRemotely { get; set; }

        #endregion

    }

}
