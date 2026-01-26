package ui;

// 2024 SSL Rules Division B 準拠
public class FieldConfig {

    // --- プレイエリア（ラインの内側のサイズ） ---
    // Division B: 9m x 6m
    public static final double FIELD_LENGTH_M = 9.0; // x方向（ゴール間）
    public static final double FIELD_WIDTH_M  = 6.0; // y方向

    // ライン幅（すべてのラインは 0.01m 幅）
    public static final double LINE_WIDTH_M = 0.01;

    // センターサークル半径（直径 1m）
    public static final double CENTER_CIRCLE_RADIUS_M = 0.5;

    // ディフェンスエリア（ルール上は「Defense Area」）
    // Division B: 2m x 1m （ゴールラインに接する長方形）
    // → ここでは、「ゴールラインから中に 1m」「上下方向に合計 2m」と解釈
    public static final double DEFENSE_AREA_DEPTH_M = 1.0; // ゴールラインからフィールド内への奥行き
    public static final double DEFENSE_AREA_WIDTH_M = 2.0; // y方向の長さ

    // ゴール幅（内側の幅）
    public static final double GOAL_WIDTH_M = 1.0;
    // （必要なら描画に使える）ゴールの奥行き・壁厚
    public static final double GOAL_DEPTH_M = 0.18;
    public static final double GOAL_WALL_THICKNESS_M = 0.02;

    // ペナルティマーク（ペナルティキック地点）
    // Division B: ゴール中心から 6m :contentReference[oaicite:1]{index=1}
    public static final double PENALTY_MARK_DISTANCE_M = 6.0;

    // ロボットの半径（直径 18cm → 半径 9cm）
    public static final double ROBOT_RADIUS_M = 0.09;
}
