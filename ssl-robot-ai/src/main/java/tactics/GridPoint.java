package tactics;

/** Simple 2D point with an associated score (higher is better). */
public final class GridPoint {
    public final double x;
    public final double y;
    public final double score;

    public GridPoint(double x, double y, double score) {
        this.x = x;
        this.y = y;
        this.score = score;
    }
}
