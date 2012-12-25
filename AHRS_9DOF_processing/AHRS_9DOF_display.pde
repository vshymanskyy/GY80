/*************************************************************************************
* Test Sketch for Razor AHRS v1.4.0
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" and "9DOF Sensor Stick"
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2011 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports and feedback:
*     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
*************************************************************************************/

import processing.opengl.*;
import processing.serial.*;

// IF THE SKETCH CRASHES OR HANGS ON STARTUP, MAKE SURE YOU ARE USING THE RIGHT SERIAL PORT:
// 1. Have a look at the Processing console output of this sketch.
// 2. Look for the serial port list and find the port you need (it's the same as in Arduino).
// 3. Set your port number here:
final static int SERIAL_PORT_NUM = 0;
// 4. Try again.

final static int SERIAL_PORT_BAUD_RATE = 115200;

float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;

float temperature = 0.0f;
float pressure = 0.0f;
float altitude = 0.0f;

float yawOffset = 0.0f;

PFont font;
Serial serial;

void drawArrow(float headWidthFactor, float headLengthFactor) {
  float headWidth = headWidthFactor * 200.0f;
  float headLength = headLengthFactor * 200.0f;
  
  pushMatrix();
  
  // Draw base
  translate(0, 0, -100);
  box(100, 100, 200);
  
  // Draw pointer
  translate(-headWidth/2, -50, -100);
  beginShape(QUAD_STRIP);
    vertex(0, 0 ,0);
    vertex(0, 100, 0);
    vertex(headWidth, 0 ,0);
    vertex(headWidth, 100, 0);
    vertex(headWidth/2, 0, -headLength);
    vertex(headWidth/2, 100, -headLength);
    vertex(0, 0 ,0);
    vertex(0, 100, 0);
  endShape();
  beginShape(TRIANGLES);
    vertex(0, 0, 0);
    vertex(headWidth, 0, 0);
    vertex(headWidth/2, 0, -headLength);
    vertex(0, 100, 0);
    vertex(headWidth, 100, 0);
    vertex(headWidth/2, 100, -headLength);
  endShape();
  
  popMatrix();
}

void drawBoard() {
  pushMatrix();

  rotateY(-radians(yaw - yawOffset));
  rotateX(-radians(pitch));
  rotateZ(radians(roll)); 

  // Board body
  fill(255, 0, 0);
  box(250, 20, 400);
  
  // Forward-arrow
  pushMatrix();
  translate(0, 0, -200);
  scale(0.5f, 0.2f, 0.25f);
  fill(0, 255, 0);
  drawArrow(1.0f, 2.0f);
  popMatrix();
    
  popMatrix();
}

// Global setup
void setup() {
  // Setup graphics
  size(640, 480, OPENGL);
  smooth();
  noStroke();
  
  // Load font
  font = loadFont("Univers-66.vlw");
  textFont(font);
  
  // Setup serial port I/O
  println("AVAILABLE SERIAL PORTS:");
  println(Serial.list());
  String portName = Serial.list()[SERIAL_PORT_NUM];
  println();
  println("HAVE A LOOK AT THE LIST ABOVE AND SET THE RIGHT SERIAL PORT NUMBER IN THE CODE!");
  println("  -> Using port " + SERIAL_PORT_NUM + ": " + portName);
  serial = new Serial(this, portName, SERIAL_PORT_BAUD_RATE);
  serial.clear();
  
  thread("ReadData");
}

void ReadData() {
  while (true) {
    byte inBuffer[] = serial.readBytesUntil(10);
    if (inBuffer != null) {
      float[] nums = float(split(new String(inBuffer), ';'));
      if (nums.length == 6) {
        if (-180 <= nums[0] && nums[0] <= 180) yaw = nums[0];
        if (-180 <= nums[1] && nums[1] <= 180) pitch = nums[1];
        if (-180 <= nums[2] && nums[2] <= 180) roll = nums[2];
        
        temperature = nums[3];
        pressure = nums[4];
        altitude = nums[5];
        
        
      }
    }
  }
}

void draw() {
  // Reset scene
  background(0);
  lights();
  
  // Draw board
  pushMatrix();
  translate(width/2, height/2, -350);
  drawBoard();
  popMatrix();
  
  textFont(font, 20);
  fill(255);
  textAlign(LEFT);

  // Output info text
  text("Press 'a' to align", 10, 25);

  // Output angles
  pushMatrix();
  translate(10, height - 10);
  textAlign(LEFT);
  text("Yaw: " + yaw, 0, 0);
  text("Pitch: " + pitch, 150, 0);
  text("Roll: " + roll, 300, 0);
  
  text("Temp: " + temperature + " C", 0, -60);
  text("Press: " + pressure + " hPa", 0, -30);
  text("Alt: " + altitude + " m", 450, 0);
  popMatrix();
}

void keyPressed() {
  switch (key) {
    case 'a':  // Align screen with Razor
      yawOffset = yaw;
  }
}



