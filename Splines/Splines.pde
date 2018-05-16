/**
 * Splines.
 *
 * Here we use the interpolator.keyFrames() nodes
 * as control points to render different splines.
 *
 * Press ' ' to change the spline mode.
 * Press 'g' to toggle grid drawing.
 * Press 'c' to toggle the interpolator path drawing.
 */

import frames.input.*;
import frames.primitives.*;
import frames.core.*;
import frames.processing.*;

// global variables
// modes: 0 natural cubic spline; 1 Hermite;
// 2 (degree 7) Bezier; 3 Cubic Bezier
int mode=0;// int mode;

Scene scene;
SplinesInterpolator interpolator;
List<OrbitNode> points;
OrbitNode eye;
boolean drawGrid = true, drawCtrl = true, drawCoordinates = true;

//Choose P3D for a 3D scene, or P2D or JAVA2D for a 2D scene
String renderer = P3D;

void setup() {
  size(800, 800, renderer);
  scene = new Scene(this);
  eye = new OrbitNode(scene);
  eye.setDamping(0);  // qué es damping
  scene.setEye(eye);
  scene.setFieldOfView(PI / 3);  // qué es Field of View
  //interactivity defaults to the eye
  scene.setDefaultGrabber(eye);  // qué es Default Grabber
  scene.setRadius(150);  // radius of the graph observed by the eye
  scene.fitBallInterpolation();  // Interpolates the eye so that the entire graph fits the screen at the end
  interpolator = new SplinesInterpolator(scene, new Frame());
  // framesjs next version, simply go:
  //interpolator = new Interpolator(scene);

  // Using OrbitNodes makes path editable
  regeneratePoints(8);
  //float scale = 2.5;
  //for (int i = 0; i < 4; i++) {
  //  Node ctrlPoint = new OrbitNode(scene);
  //  ctrlPoint.setPosition(new Vector(i*scale, pow(i*scale, 2), 0));
  //  interpolator.addKeyFrame(ctrlPoint);
  //}
}

void draw() {
  background(175);
  if (drawGrid) {
    stroke(255, 255, 0);
    scene.drawGrid(200, 50);  // draws a grid of size onto pGraphics in the XY plane, centered on (0,0,0), having subdivisions
  }
  if (drawCtrl) {
    fill(255, 0, 0);
    stroke(255, 0, 255);
    for (Frame frame : interpolator.keyFrames())  // returns the list of key frames which defines this interpolator
      scene.drawPickingTarget((Node)frame);  // Draws the node picking target: a shooter target visual hint of Node.precisionThreshold()
  } else {
    fill(255, 0, 0);
    stroke(255, 0, 255);
    scene.drawPath(interpolator);
  }
  // implement me
  // draw curve according to control polygon an mode
  // To retrieve the positions of the control points do:
  // for(Frame frame : interpolator.keyFrames())
  //   frame.position();
  //switch (mode) {
  //  case 0:
  //    //scene.drawNaturalPath(interpolator);
  //    break;
  //  case 1:
  //    break;
  //  case 2:
  //    break;
  //  case 3:
  //    break;
  //}
  
  //if (drawCoordinates) {
  //  for (Frame frame : interpolator.myPath()) {
  //    pushStyle();
  //    textSize(8);
  //    text((int) frame.position().x() + ", " + 
  //         (int) frame.position().y() + ", " +
  //         (int) frame.position().z() + ", ", frame.position().x(), frame.position().y(), frame.position().z());
  //    popStyle();
  //  }
  //}
}

void regeneratePoints(int numOfPoints) {
  points = new ArrayList<OrbitNode>();

  for (int i = 0; i < numOfPoints; i++) {
    OrbitNode ctrlPoint = new OrbitNode(scene);
    ctrlPoint.randomize();  // returns a random graph node. The node is randmly positioned inside the ball defined by center and raduis

    points.add(ctrlPoint);
    interpolator.addKeyFrame(ctrlPoint);  // appends a new key frame to path
  }
}

void keyPressed() {
  if (key == ' ')
    mode = mode < 3 ? mode+1 : 0;
  if (key == 'g')
    drawGrid = !drawGrid;
  if (key == 'r')
    regeneratePoints(8);
  if (key == 'c')
    drawCtrl = !drawCtrl;
  if (key == 'x') {
    drawCoordinates = !drawCoordinates;
  }
}
