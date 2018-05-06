import frames.primitives.Frame;
import frames.primitives.Quaternion;
import frames.primitives.Vector;
import frames.core.Interpolator;

import java.util.List;
import java.util.Arrays;

public class myInterpolator extends Interpolator {
  int numSubDivisions = 7;
  
  protected List<Frame> _path;
  
  public myInterpolator(Graph graph, Frame frame) {
    super(graph, frame);
  }
  
  @Override
  public List<Frame> path() {
    
    switch (mode) {
      case 0:
        _naturalCubicSplinePath();
        break;
      case 1:
        break;
      case 2:
        break;
      case 3:
        break;
    }
    return _path;
  }
  
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
  
  //protected List<Vector> getu(List<Vector> v) {
  //  List<Vector> u = new ArrayList();
    
  //  int n = this.keyFrames().size();
    
  //  for (int i = 0; i <= n); i++) {
  //    u.add(new Vector(1 - v.get(i).x(),
  //                     1 - v.get(i).y(), 
  //                     1 - v.get(i).z()));
  //  }
  //  return u;
  //}
  
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
           zi/2*pow((xyz-xyzj),2) + 
           (zj-zi)/(6*h)*pow((xyz-xyzj),3);
  }
  
  //protected List<Vector> updatet() {
  //  int n = this.keyFrames().size();
    
  //  List<Vector> t = Arrays.asList(new Vector[n + 1]);
    
  //  for (int i = 0; i < t.size(); i++) {
  //    t.set(i, new Vector(1, 1, 1));
  //  }
    
  //  return t;
  //}
}
