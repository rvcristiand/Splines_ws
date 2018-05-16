import frames.primitives.Frame;
import frames.primitives.Quaternion;
import frames.primitives.Vector;
import frames.core.Interpolator;

import java.util.List;
import java.util.Arrays;

public class SplinesInterpolator extends Interpolator {
  int numSubDivisions = 21;

  float hermiteSections = 50f;
  float nBezierSections = 100f;

  float flatnessFactor = 1.001;

  protected List<Frame> _path;

  public SplinesInterpolator(Graph graph, Frame frame) {
    super(graph, frame);
  }

  @Override
  public List<Frame> path() {
    switch (mode) {
      case 0:
        _naturalCubicSplinePath();
        break;
      case 1:
        _hermiteSplinePath();
        break;
      case 2:
        _nBezierSplinePath();
        break;
      case 3:
        _cubicBezierSplinePath();
        break;
      default:
        _updatePath();
        break;
    }

    return _path;
  }

  /**
   * Natural Cubic Spline
   */
  protected void _naturalCubicSplinePath() {
    int n = this.keyFrames().size() - 1;

    _path = Arrays.asList(new Frame[numSubDivisions*n + 1]);

    List<Vector> z = Arrays.asList(new Vector[n+1]);

    List<Vector> h = this._geth();
    List<Vector> v = this._getv();
    List<Vector> u = this._getu();
    List<Vector> d = this._getd();

    List<Vector> q = Arrays.asList(new Vector[n+1]);
    List<Vector> t = Arrays.asList(new Vector[n+1]);
    Vector p;

    q.set(0, Vector.divide(v.get(0), -2));
    t.set(0, Vector.divide(d.get(0), 2));

    for (int i = 1; i <= n; i++) {
      p = new Vector(u.get(i).x()*q.get(i-1).x() + 2, 
        u.get(i).y()*q.get(i-1).y() + 2, 
        u.get(i).z()*q.get(i-1).z() + 2);

      q.set(i, new Vector(-v.get(i).x()/p.x(), 
        -v.get(i).y()/p.y(), 
        -v.get(i).z()/p.z()));

      t.set(i, new Vector((d.get(i).x() - u.get(i).x()*t.get(i-1).x()) / p.x(), 
        (d.get(i).y() - u.get(i).y()*t.get(i-1).y()) / p.y(), 
        (d.get(i).z() - u.get(i).z()*t.get(i-1).z()) / p.z()));
    }

    z.set(n, new Vector(0, 0, 0));

    for (int i = n - 1; i > 0; i--) {
      z.set(i, new Vector(q.get(i).x()*z.get(i+1).x() + t.get(i).x(), 
        q.get(i).y()*z.get(i+1).y() + t.get(i).y(), 
        q.get(i).z()*z.get(i+1).z() + t.get(i).z()));
    }

    z.set(0, new Vector(0, 0, 0));

    Vector xyzi = new Vector(0, 0, 0);
    Vector xyz = new Vector(0, 0, 0);
    Vector paso;

    for (int i = 0; i < n; i++) {
      paso = Vector.divide(h.get(i), numSubDivisions);

      for (int j = 0; j < numSubDivisions; j++) {

        this._path.set(i*numSubDivisions + j, new Frame(_s(xyz, xyzi, h.get(i), 
          this.keyFrames().get(i+0).position(), 
          this.keyFrames().get(i+1).position(), 
          z.get(i+0), z.get(i+1)), 
          new Quaternion()));

        xyz.add(paso);
      }
      xyzi.add(new Vector(1, 1, 1));
    }
    this._path.set(numSubDivisions*n, new Frame(this.keyFrames().get(n).position(), 
      new Quaternion()));
  }

  protected List<Vector> _geth() {
    int n = this.keyFrames().size() - 1;

    List<Vector> _h = Arrays.asList(new Vector[n]);

    for (int i = 0; i < n; i++) {
      //h.add(new Vector(this.keyFrames().get(i + 1).position().x() - this.keyFrames().get(i).position().x(), 
      //                 this.keyFrames().get(i + 1).position().y() - this.keyFrames().get(i).position().y(),
      //                 this.keyFrames().get(i + 1).position().z() - this.keyFrames().get(i).position().z()));
      _h.set(i, new Vector(1, 1, 1));
    }

    return _h;
  }

  protected List<Vector> _getv() {
    int n = this.keyFrames().size() - 1;

    List<Vector> _v = Arrays.asList(new Vector[n+1]);

    _v.set(0, new Vector(0, 0, 0));

    for (int i = 1; i < n; i++) {
      //v.add(new Vector(h.get(i).x() / (h.get(i).x() + h.get(i + 1).x()),
      //                 h.get(i).y() / (h.get(i).y() + h.get(i + 1).y()),
      //                 h.get(i).z() / (h.get(i).z() + h.get(i + 1).z())));
      _v.set(i, new Vector(0.5, 0.5, 0.5));
    }

    _v.set(n, new Vector(0, 0, 0));

    return _v;
  }

  protected List<Vector> _getu() {
    int n = this.keyFrames().size() - 1;

    List<Vector> _u = Arrays.asList(new Vector[n+1]);

    _u.set(0, new Vector(0, 0, 0));

    for (int i = 1; i < n; i++) {
      _u.set(i, new Vector(0.5, 0.5, 0.5));
    }

    _u.set(n, new Vector(0, 0, 0));

    return _u;
  }

  protected List<Vector> _getd() {
    int n = this.keyFrames().size() - 1;

    List<Vector> _d = Arrays.asList(new Vector[n+1]);

    _d.set(0, new Vector(0, 0, 0));

    for (int i = 1; i < n; i++) {
      _d.set(i, new Vector(3 * (this.keyFrames().get(i+1).position().x() - 
        2 * this.keyFrames().get(i).position().x() + 
        this.keyFrames().get(i-1).position().x()), 
        3 * (this.keyFrames().get(i+1).position().y() - 
        2 * this.keyFrames().get(i).position().y() + 
        this.keyFrames().get(i-1).position().y()), 
        3 * (this.keyFrames().get(i+1).position().z() - 
        2 * this.keyFrames().get(i).position().z() + 
        this.keyFrames().get(i-1).position().z())));
    }

    _d.set(n, new Vector(0, 0, 0));

    return _d;
  }

  protected Vector _s(Vector xyz, Vector xyzj, Vector h, Vector fi, Vector fj, Vector zi, Vector zj) {

    return new Vector(_s(xyz.x(), xyzj.x(), h.x(), fi.x(), fj.x(), zi.x(), zj.x()), 
      _s(xyz.y(), xyzj.y(), h.y(), fi.y(), fj.y(), zi.y(), zj.y()), 
      _s(xyz.z(), xyzj.z(), h.z(), fi.z(), fj.z(), zi.z(), zj.z()));
  }

  protected float _s(float xyz, float xyzj, float h, float fi, float fj, float zi, float zj) {

    return fi + 
      ((fj-fi)/h-h/6*(2*zi+zj))*(xyz-xyzj) + 
      zi/2*pow((xyz-xyzj), 2) + 
      (zj-zi)/(6*h)*pow((xyz-xyzj), 3);
  }

  /**
   * Hermite Spline
   */
   
  protected void _hermiteSplinePath() {
    _path = new ArrayList<Frame>();
    
    List<Frame> kf = keyFrames();

    for (int i = 0; i < kf.size() - 1; i++) {
      Vector pk = kf.get(i).position();
      Vector pk1 = kf.get(i + 1).position();

      float[][] tangents = this._calculateTangents(kf, i);
      float[] dpk = tangents[0];
      float[] dpk1 = tangents[1];

      for (float s = 0; s < hermiteSections; s += 1) {
        float t1 = s / hermiteSections;
        float t2 = t1 * t1;
        float t3 = t1 * t1 * t1;

        float x = (2 * t3  - 3 * t2 + 1) * pk.x() + (-2 * t3 + 3 * t2) * pk1.x() + (t3 - 2 * t2 + t1) * dpk[0] + (t3 - t2) * dpk1[0];
        float y = (2 * t3  - 3 * t2 + 1) * pk.y() + (-2 * t3 + 3 * t2) * pk1.y() + (t3 - 2 * t2 + t1) * dpk[1] + (t3 - t2) * dpk1[1];
        float z = (2 * t3  - 3 * t2 + 1) * pk.z() + (-2 * t3 + 3 * t2) * pk1.z() + (t3 - 2 * t2 + t1) * dpk[2] + (t3 - t2) * dpk1[2];

        Frame frm = new Frame(new Vector(x, y, z), new Quaternion(0, 0, 0));
        _path.add(frm);
      }
    }
  }

  private float[][] _calculateTangents(List<Frame> kf, int i) {
    float[][] m = new float[2][3];

    Vector kfi = kf.get(i).position();
    Vector kfi1 = kf.get(i + 1).position();

    if (i == 0) {
      Vector kfi2 = kf.get(i + 2).position();

      m[0][0] = kfi1.x() - kfi.x();
      m[0][1] = kfi1.y() - kfi.y();
      m[0][2] = kfi1.z() - kfi.z();

      m[1][0] = (kfi2.x() - kfi.x()) / 2;
      m[1][1] = (kfi2.y() - kfi.y()) / 2;
      m[1][2] = (kfi2.z() - kfi.z()) / 2;
    } else if (i == kf.size() - 2) {
      Vector kfi_1 = kf.get(i - 1).position();

      m[0][0] = (kfi1.x() - kfi_1.x()) / 2;
      m[0][1] = (kfi1.y() - kfi_1.y()) / 2;
      m[0][2] = (kfi1.z() - kfi_1.z()) / 2;

      m[1][0] = kfi1.x() - kfi.x();
      m[1][1] = kfi1.y() - kfi.y();
      m[1][2] = kfi1.z() - kfi.z();
    } else {
      Vector kfi_1 = kf.get(i - 1).position();
      Vector kfi2 = kf.get(i + 2).position();

      m[0][0] = (kfi1.x() - kfi_1.x()) / 2;
      m[0][1] = (kfi1.y() - kfi_1.y()) / 2;
      m[0][2] = (kfi1.z() - kfi_1.z()) / 2;

      m[1][0] = (kfi2.x() - kfi.x()) / 2;
      m[1][1] = (kfi2.y() - kfi.y()) / 2;
      m[1][2] = (kfi2.z() - kfi.z()) / 2;
    }

    return m;
  }
  
  /**
   * N Bezier Spline
   */
  protected void _nBezierSplinePath() {
    _path = new ArrayList<Frame>();

    List<Vector> points = new ArrayList<Vector>();
      for (Frame frm : keyFrames()) {
        points.add(frm.position());
      }

    for (float s = 0; s < nBezierSections; s += 1) {
      float t = s / nBezierSections;

      List<Vector> kf = new ArrayList<Vector>(points);

      while (kf.size() > 1) {
        List<Vector> newKf = new ArrayList<Vector>();

        for (int i = 0; i < kf.size() - 1; i++) {
          newKf.add(_pointInLine(kf.get(i), kf.get(i + 1), t));
        }

        kf = newKf;
      }

      _path.add(new Frame(kf.get(0), new Quaternion(0, 0, 0)));
    }
  }

  private Vector _pointInLine(Vector p0, Vector p1, float t) {
    float x = p0.x() + (p1.x() - p0.x()) * t;
    float y = p0.y() + (p1.y() - p0.y()) * t;
    float z = p0.z() + (p1.z() - p0.z()) * t;

    return new Vector(x, y, z);
  }

  /**
   * Cubic Bezier Spline
   */
  protected void _cubicBezierSplinePath() {
    _path = new ArrayList<Frame>();

    List<Frame> b = keyFrames();
    int n = b.size() - 1;

    List<List<Vector>> p = new ArrayList<List<Vector>>(); // List<List<Vector>> p = Arrays.asList(Arrays.asList(new Vector[4])[n]);
    
    List<Vector> pi = Arrays.asList(new Vector[4]);
    Vector paso;

    paso = Vector.divide(Vector.subtract(b.get(1).position(), b.get(0).position()), 3);
    pi.set(0, b.get(0).position());
    pi.set(1, Vector.add(b.get(0).position(), Vector.multiply(paso, 1)));
    pi.set(2, Vector.add(b.get(0).position(), Vector.multiply(paso, 2)));
    paso = Vector.divide(Vector.subtract(b.get(2).position(), b.get(1).position()), 3);
    pi.set(3, Vector.add(pi.get(2), Vector.divide(Vector.subtract(Vector.add(b.get(1).position(), paso), pi.get(2)), 2)));

    p.add(pi); // p.set(0, pi);

    for (int i = 1; i < n - 1; i++) {
      pi = Arrays.asList(new Vector[4]);
      paso = Vector.divide(Vector.subtract(b.get(i+1).position(), b.get(i).position()), 3);

      pi.set(0, p.get(i-1).get(3));
      pi.set(1, Vector.add(b.get(i).position(), Vector.multiply(paso, 1)));
      pi.set(2, Vector.add(b.get(i).position(), Vector.multiply(paso, 2)));
      paso = Vector.divide(Vector.subtract(b.get(i+2).position(), b.get(i+1).position()), 3);
      pi.set(3, Vector.add(pi.get(2), Vector.divide(Vector.subtract(Vector.add(b.get(i+1).position(), paso), pi.get(2)), 2)));

      p.add(pi); // p.set(i, pi);
    }

    paso = Vector.divide(Vector.subtract(b.get(n).position(), b.get(n-1).position()), 3);
    pi = Arrays.asList(new Vector[4]);
    pi.set(0, p.get(n-2).get(3));
    pi.set(1, Vector.add(b.get(n-1).position(), Vector.multiply(paso, 1)));
    pi.set(2, Vector.add(b.get(n-1).position(), Vector.multiply(paso, 2)));
    pi.set(3, b.get(n).position());

    p.add(pi); // p.set(n - 1, pi);
    
    for (List<Vector> _p : p) {
      _cubicBezier(_p); 
    }
    _path.add(new Frame(b.get(n).position(), new Quaternion()));
  }

  protected void _cubicBezier(List<Vector> p) {

    boolean isflat = (Vector.distance(p.get(0), p.get(1)) + 
      Vector.distance(p.get(1), p.get(2)) + 
      Vector.distance(p.get(2), p.get(3))) < 
      (flatnessFactor*Vector.distance(p.get(0), p.get(3)));

    if (isflat) {
      _path.add(new Frame(p.get(0), new Quaternion()));
    } else {
      Vector q0, q1, q2, r0, r1, s0;
      List<Vector> p0 = Arrays.asList(new Vector[4]);
      List<Vector> p1 = Arrays.asList(new Vector[4]);

      q0 = Vector.multiply(Vector.add(p.get(0), p.get(1)), 0.5); // println("q0", q0);
      q1 = Vector.multiply(Vector.add(p.get(1), p.get(2)), 0.5); // println("q1", q1);
      q2 = Vector.multiply(Vector.add(p.get(2), p.get(3)), 0.5); // println("q2", q2);

      r0 = Vector.multiply(Vector.add(q0, q1), 0.5); // println("r0", r0);
      r1 = Vector.multiply(Vector.add(q1, q2), 0.5); // println("r1", r1);

      s0 = Vector.multiply(Vector.add(r0, r1), 0.5); // println("s0", s0);

      p0.set(0, p.get(0));
      p0.set(1, q0);
      p0.set(2, r0);
      p0.set(3, s0);

      p1.set(0, s0);
      p1.set(1, r1);
      p1.set(2, q2);
      p1.set(3, p.get(3));

      _cubicBezier(p0);
      _cubicBezier(p1);
    }
  }
}
