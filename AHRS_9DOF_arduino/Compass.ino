/* This file is part of the Razor AHRS Firmware */

void Compass_Heading()
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Tilt compensated magnetic field X
  mag_x = Magn_Vector[0]*cos_pitch + Magn_Vector[1]*sin_roll*sin_pitch + Magn_Vector[2]*cos_roll*sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = Magn_Vector[1]*cos_roll - Magn_Vector[2]*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-mag_y, mag_x);
}
