package ui;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import javax.swing.*;
import world.Ball;
import world.Robot;
import world.WorldState;

public class FieldPanel extends JPanel {

    private WorldState world;   // ← 追加

    private double scale;
    private double centerX;
    private double centerY;

    // WorldState を受け取るコンストラクタに変更
    public FieldPanel(WorldState world) {
        this.world = world;
        setBackground(new Color(20, 120, 20));
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                            RenderingHints.VALUE_ANTIALIAS_ON);

        int w = getWidth();
        int h = getHeight();

        double sx = w / FieldConfig.FIELD_LENGTH_M;
        double sy = h / FieldConfig.FIELD_WIDTH_M;
        scale = Math.min(sx, sy) * 0.9;

        centerX = w / 2.0;
        centerY = h / 2.0;

        double halfL = FieldConfig.FIELD_LENGTH_M / 2.0;
        double halfW = FieldConfig.FIELD_WIDTH_M / 2.0;

        g2.setColor(Color.WHITE);
        float linePx = (float) (FieldConfig.LINE_WIDTH_M * scale);
        if (linePx < 1f) linePx = 1f;
        g2.setStroke(new BasicStroke(linePx));

        // ----- フィールドライン -----
        drawRectByField(g2, -halfL, -halfW, halfL, halfW);
        drawLineByField(g2, 0.0, -halfW, 0.0, halfW);
        drawCircleByField(g2, 0.0, 0.0, FieldConfig.CENTER_CIRCLE_RADIUS_M);
        drawPenaltyArea(g2, -halfL, true);
        drawPenaltyArea(g2,  halfL, false);
        drawGoalLines(g2, -halfL, true);
        drawGoalLines(g2,  halfL, false);

        // ----- ロボット描画 -----
        drawRobots(g2);

        // ----- ボール描画 -----
        drawBall(g2);

        // ----- Debug: draw suggested target points (from Main's tactical layer) -----
        drawDebugTargets(g2);

        // ----- Debug: draw per-robot planned targets + ids -----
        drawPlannedTargetsAndIds(g2);

        // ----- Debug: draw man-mark targets (defense assignment) -----
        drawMarkTargets(g2);

        // ----- Scoreboard (top-center) -----
        drawScoreboard(g2);
    }

    private void drawScoreboard(Graphics2D g2) {
        // Pull score from sim.Main via reflection to avoid hard dependency direction.
        int blue = 0;
        int red = 0;
        try {
            Class<?> main = Class.forName("sim.Main");
            java.lang.reflect.Method m = main.getMethod("getScore");
            Object o = m.invoke(null);
            if (o instanceof int[]) {
                int[] s = (int[]) o;
                if (s.length >= 2) {
                    blue = s[0];
                    red = s[1];
                }
            }
        } catch (Throwable ignored) {
            // Optional overlay only.
        }

        String text = "BLUE " + blue + "  -  " + red + " RED";

        Font base = g2.getFont();
        Font f = base.deriveFont(Font.BOLD, 18f);
        g2.setFont(f);

        FontMetrics fm = g2.getFontMetrics();
        int tw = fm.stringWidth(text);
        int th = fm.getHeight();

        int padX = 12;
        int padY = 6;
        int x = (getWidth() - tw) / 2;
        int y = 20; // near top

        // Background
        g2.setColor(new Color(0, 0, 0, 140));
        g2.fillRoundRect(x - padX, y - fm.getAscent() - padY, tw + padX * 2, th + padY * 2, 10, 10);

        // Text
        g2.setColor(Color.WHITE);
        g2.drawString(text, x, y);

        g2.setFont(base);
    }

    private void drawMarkTargets(Graphics2D g2) {
        if (world == null) return;
        try {
            Class<?> mainClz = Class.forName("sim.Main");
            java.lang.reflect.Method m = mainClz.getMethod("getMarkTargets");
            Object raw = m.invoke(null);
            if (!(raw instanceof double[][])) return;
            double[][] marks = (double[][]) raw;

            Stroke old = g2.getStroke();
            g2.setStroke(new BasicStroke(1.0f));

            // Blue
            if (world.ourRobots != null) {
                for (Robot r : world.ourRobots) {
                    if (r == null) continue;
                    int id = r.id;
                    if (id < 0 || id >= marks.length) continue;
                    if (marks[id] == null) continue;

                    double mx = marks[id][0];
                    double my = marks[id][1];
                    int x1 = fieldToScreenX(r.x);
                    int y1 = fieldToScreenY(r.y);
                    int x2 = fieldToScreenX(mx);
                    int y2 = fieldToScreenY(my);

                    g2.setColor(new Color(0, 120, 255, 70));
                    g2.drawLine(x1, y1, x2, y2);
                    g2.setColor(new Color(0, 120, 255, 140));
                    drawCross(g2, x2, y2, 6);
                }
            }

            // Red
            if (world.oppRobots != null) {
                for (Robot r : world.oppRobots) {
                    if (r == null) continue;
                    int id = r.id;
                    if (id < 0 || id >= marks.length) continue;
                    if (marks[id] == null) continue;

                    double mx = marks[id][0];
                    double my = marks[id][1];
                    int x1 = fieldToScreenX(r.x);
                    int y1 = fieldToScreenY(r.y);
                    int x2 = fieldToScreenX(mx);
                    int y2 = fieldToScreenY(my);

                    g2.setColor(new Color(255, 60, 60, 70));
                    g2.drawLine(x1, y1, x2, y2);
                    g2.setColor(new Color(255, 60, 60, 140));
                    drawCross(g2, x2, y2, 6);
                }
            }

            g2.setStroke(old);
        } catch (Throwable ignore) {
            // Optional overlay only.
        }
    }

    private void drawCross(Graphics2D g2, int x, int y, int r) {
        g2.drawLine(x - r, y - r, x + r, y + r);
        g2.drawLine(x - r, y + r, x + r, y - r);
    }

    private void drawPlannedTargetsAndIds(Graphics2D g2) {
        // Draw robot id labels and their intended destinations.
        drawRobotIds(g2);
        drawPlannedTargets(g2);
    }

    private void drawRobotIds(Graphics2D g2) {
        if (world == null) return;

        Font base = g2.getFont();
        Font f = base.deriveFont(Font.BOLD, (float) Math.max(11.0, 10.0 * scale / 80.0));
        g2.setFont(f);

        // Blue ids
        if (world.ourRobots != null) {
            for (Robot r : world.ourRobots) {
                if (r == null) continue;
                int cx = fieldToScreenX(r.x);
                int cy = fieldToScreenY(r.y);
                g2.setColor(Color.WHITE);
                String s = String.valueOf(r.id);
                g2.drawString(s, cx - 6, cy + 4);
            }
        }

        // Red ids
        if (world.oppRobots != null) {
            for (Robot r : world.oppRobots) {
                if (r == null) continue;
                int cx = fieldToScreenX(r.x);
                int cy = fieldToScreenY(r.y);
                g2.setColor(Color.WHITE);
                String s = String.valueOf(r.id);
                g2.drawString(s, cx - 6, cy + 4);
            }
        }
        g2.setFont(base);
    }

    @SuppressWarnings("unchecked")
    private void drawPlannedTargets(Graphics2D g2) {
        try {
            Class<?> main = Class.forName("sim.Main");
            java.lang.reflect.Method m = main.getDeclaredMethod("getPlannedTargets");
            Object o = m.invoke(null);
            if (!(o instanceof double[][])) return;
            double[][] targets = (double[][]) o;

            // Draw faint lines from each robot to its planned target.
            g2.setStroke(new BasicStroke(1.5f));

            // Blue
            if (world.ourRobots != null) {
                for (Robot r : world.ourRobots) {
                    if (r == null) continue;
                    int id = r.id;
                    if (id < 0 || id >= targets.length) continue;
                    double[] t = targets[id];
                    if (t == null) continue;

                    int x1 = fieldToScreenX(r.x);
                    int y1 = fieldToScreenY(r.y);
                    int x2 = fieldToScreenX(t[0]);
                    int y2 = fieldToScreenY(t[1]);

                    g2.setColor(new Color(80, 180, 255, 110));
                    g2.drawLine(x1, y1, x2, y2);
                    g2.drawOval(x2 - 6, y2 - 6, 12, 12);
                }
            }

            // Red
            if (world.oppRobots != null) {
                for (Robot r : world.oppRobots) {
                    if (r == null) continue;
                    int id = r.id;
                    if (id < 0 || id >= targets.length) continue;
                    double[] t = targets[id];
                    if (t == null) continue;

                    int x1 = fieldToScreenX(r.x);
                    int y1 = fieldToScreenY(r.y);
                    int x2 = fieldToScreenX(t[0]);
                    int y2 = fieldToScreenY(t[1]);

                    g2.setColor(new Color(255, 120, 120, 110));
                    g2.drawLine(x1, y1, x2, y2);
                    g2.drawOval(x2 - 6, y2 - 6, 12, 12);
                }
            }
        } catch (Throwable ignored) {
            // No-op
        }
    }

    private void drawDebugTargets(Graphics2D g2) {
        // Avoid hard dependency on sim.Main at compile time via fully qualified name.
        // If the fields/methods don't exist, this remains a no-op.
        try {
            Class<?> main = Class.forName("sim.Main");
            java.lang.reflect.Method m = main.getDeclaredMethod("getDebugTargets");
            Object o = m.invoke(null);
            if (!(o instanceof double[])) return;
            double[] arr = (double[]) o;
            if (arr.length < 4) return;

            // arr = { blueX, blueY, redX, redY }
            int bx = fieldToScreenX(arr[0]);
            int by = fieldToScreenY(arr[1]);
            int rx = fieldToScreenX(arr[2]);
            int ry = fieldToScreenY(arr[3]);

            g2.setStroke(new BasicStroke(2f));

            g2.setColor(new Color(80, 180, 255));
            g2.drawOval(bx - 8, by - 8, 16, 16);
            g2.drawLine(bx - 10, by, bx + 10, by);
            g2.drawLine(bx, by - 10, bx, by + 10);

            g2.setColor(new Color(255, 120, 120));
            g2.drawOval(rx - 8, ry - 8, 16, 16);
            g2.drawLine(rx - 10, ry, rx + 10, ry);
            g2.drawLine(rx, ry - 10, rx, ry + 10);
        } catch (Throwable ignored) {
            // No-op
        }
    }

    private void drawBall(Graphics2D g2) {
        Ball ball = world.ball;
        if (ball == null) return;

        double ballRadiusM = 0.0215; // SSL ball radius ~ 21.5mm
        int cx = fieldToScreenX(ball.x);
        int cy = fieldToScreenY(ball.y);
        int rPx = (int) Math.round(ballRadiusM * scale);

        g2.setColor(Color.ORANGE);
        g2.fill(new Ellipse2D.Double(cx - rPx, cy - rPx, rPx * 2, rPx * 2));
        g2.setColor(Color.BLACK);
        g2.draw(new Ellipse2D.Double(cx - rPx, cy - rPx, rPx * 2, rPx * 2));
    }

    // ロボットを描く処理
    private void drawRobots(Graphics2D g2) {
        double robotRadiusM = 0.09; // SSL ロボット半径 ~9cm くらいのイメージ

        // 自チーム（青）
        g2.setColor(Color.BLUE);
        for (Robot r : world.ourRobots) {
            drawRobot(g2, r, robotRadiusM);
        }

        // 相手チーム（赤）
        g2.setColor(Color.RED);
        for (Robot r : world.oppRobots) {
            drawRobot(g2, r, robotRadiusM);
        }
    }

    private void drawRobot(Graphics2D g2, Robot r, double radiusM) {
        int cx = fieldToScreenX(r.x);
        int cy = fieldToScreenY(r.y);
        int rPx = (int) Math.round(radiusM * scale);

        g2.fill(new Ellipse2D.Double(cx - rPx, cy - rPx, rPx * 2, rPx * 2));
    }


    /** フィールド座標系(x,y) -> 画面ピクセルX */
    private int fieldToScreenX(double xField) {
        // x: 左がマイナス、右がプラス → ピクセルでは右がプラス
        return (int) Math.round(centerX + xField * scale);
    }

    /** フィールド座標系(x,y) -> 画面ピクセルY */
    private int fieldToScreenY(double yField) {
        // y: 上がプラス、下がマイナス（SSLの典型）と仮定するので、画面では逆転させる
        return (int) Math.round(centerY - yField * scale);
    }

    /** フィールド座標で長方形を描く（左下と右上を指定） */
    private void drawRectByField(Graphics2D g2,
                                 double xMin, double yMin,
                                 double xMax, double yMax) {
        int x1 = fieldToScreenX(xMin);
        int y1 = fieldToScreenY(yMax); // 注意：yは上下反転
        int x2 = fieldToScreenX(xMax);
        int y2 = fieldToScreenY(yMin);

        int w = x2 - x1;
        int h = y2 - y1;

        g2.draw(new Rectangle2D.Double(x1, y1, w, h));
    }

    /** フィールド座標で線分を描く */
    private void drawLineByField(Graphics2D g2,
                                 double x1Field, double y1Field,
                                 double x2Field, double y2Field) {
        int x1 = fieldToScreenX(x1Field);
        int y1 = fieldToScreenY(y1Field);
        int x2 = fieldToScreenX(x2Field);
        int y2 = fieldToScreenY(y2Field);
        g2.drawLine(x1, y1, x2, y2);
    }

    /** フィールド座標で円を描く（中心と半径） */
    private void drawCircleByField(Graphics2D g2,
                                   double cxField, double cyField,
                                   double radiusField) {
        int cx = fieldToScreenX(cxField);
        int cy = fieldToScreenY(cyField);
        int rPx = (int) Math.round(radiusField * scale);

        g2.draw(new Ellipse2D.Double(cx - rPx, cy - rPx, rPx * 2, rPx * 2));
    }

    /** ディフェンスエリアの描画（Rule: 2m x 1m） */
    private void drawDefenseArea(Graphics2D g2, double goalLineX, boolean isOurSide) {
        double halfW = FieldConfig.DEFENSE_AREA_WIDTH_M / 2.0;

        double xMin, xMax;
        if (isOurSide) {
            // 左側ゴール前: goalLineX から＋方向（右）に広がる
            xMin = goalLineX;
            xMax = goalLineX + FieldConfig.DEFENSE_AREA_DEPTH_M;
        } else {
            // 右側ゴール前: goalLineX から－方向（左）に広がる
            xMin = goalLineX - FieldConfig.DEFENSE_AREA_DEPTH_M;
            xMax = goalLineX;
        }

        drawRectByField(g2,
                xMin, -halfW,
                xMax,  halfW);
    }

    /** ゴール枠（幅・奥行き）を描画 */
    private void drawGoalLines(Graphics2D g2, double goalLineX, boolean isOurSide) {
        double halfGoal = FieldConfig.GOAL_WIDTH_M / 2.0;
        double depth = FieldConfig.GOAL_DEPTH_M;

        // Mouth line on the goal line
        drawLineByField(g2, goalLineX, -halfGoal, goalLineX, halfGoal);

        // Frame rectangle (outside the field)
        double xMin;
        double xMax;
        if (isOurSide) {
            // Left goal: extends to negative x
            xMin = goalLineX - depth;
            xMax = goalLineX;
        } else {
            // Right goal: extends to positive x
            xMin = goalLineX;
            xMax = goalLineX + depth;
        }

        Stroke old = g2.getStroke();
        g2.setStroke(new BasicStroke(Math.max(2f, (float) (FieldConfig.GOAL_WALL_THICKNESS_M * scale))));
        drawRectByField(g2, xMin, -halfGoal, xMax, halfGoal);
        g2.setStroke(old);
    }

    /** ペナルティマーク（小さな点） */
    private void drawPenaltyMark(Graphics2D g2, double goalLineX, boolean isOurSide) {
        double sign = isOurSide ? 1.0 : -1.0;
        double x = goalLineX + sign * FieldConfig.PENALTY_MARK_DISTANCE_M;
        double y = 0.0;

        int px = fieldToScreenX(x);
        int py = fieldToScreenY(y);
        int rPx = (int) Math.round(0.03 * scale); // 3cmくらい

        g2.fill(new Ellipse2D.Double(px - rPx, py - rPx, rPx * 2, rPx * 2));
    }

    // ===== ここからロボット描画関連 =====

    /** ペナルティエリア（ゴール前の矩形）とペナルティマークを描く */
    private void drawPenaltyArea(Graphics2D g2, double goalLineX, boolean isOurSide) {
        // ペナルティエリアはディフェンスエリアの矩形表示と
        // ペナルティマーク（小さな点）の表示を組み合わせる
        drawDefenseArea(g2, goalLineX, isOurSide);
        drawPenaltyMark(g2, goalLineX, isOurSide);
    }
}
