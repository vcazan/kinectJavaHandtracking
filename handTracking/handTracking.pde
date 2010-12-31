import org.openkinect.*;
import org.openkinect.processing.*;
import processing.serial.*;
import processing.video.*;


Kinect kinect;
DepthImage imgd;

boolean depth = true;
boolean rgb = false;

Serial myPort;       
int intence;
void setup() {
  size(640,520);
  myPort = new Serial(this, Serial.list()[0], 9600);


  noStroke();
  smooth();
imgd = new DepthImage(this);


  kinect = new Kinect(this);
  kinect.start();
  kinect.enableDepth(depth);
  kinect.enableRGB(rgb);
}

void draw() {
 // background(0);
  image(kinect.getRGBImage(),0,0);
  //image(kinect.getDepthImage(),640,0);
  fill(255);
  text("RGB FPS: " + kinect.getRGBFPS(),10,495);
  text("DEPTH FPS: " + kinect.getDepthFPS(),640,495);
  text("Press 'd' to enable/disable depth    Press 'r' to enable/disable rgb image    Framerate: " + frameRate,10,515);
  int brightestX = 0; // X-coordinate of the brightest video pixel
    int brightestY = 0; // Y-coordinate of the brightest video pixel
    float brightestValue = 0; // Brightness of the brightest video pixel
    // Search for the brightest pixel: For each row of pixels in the video image and
    // for each pixel in the yth row, compute each pixel's index in the video
   
    PImage video = kinect.getDepthImage();
    
    image(video, 0, 0);
    
    int index = 0;
    for (int y = 0; y < video.height; y++) {
      for (int x = 0; x < video.width; x++) {
        // Get the color stored in the pixel
        int pixelValue = video.pixels[index];
        // Determine the brightness of the pixel
        float pixelBrightness = brightness(pixelValue);
        // If that value is brighter than any previous, then store the
        // brightness of that pixel, as well as its (x,y) location
        if (pixelBrightness > brightestValue) {
          brightestValue = pixelBrightness;
          brightestY = y;
          brightestX = x;
        }
        index++;
      }
    }
 
    // Draw a large, yellow circle at the brightest pixel
    fill(255, 204, 0, 128);
    ellipse(brightestX, brightestY, 20, 20);
    
    println(brightestX);
    if (brightestX > 400){
      myPort.write(65);
    }

}

void keyPressed() {
  if (key == 'd') {
    depth = !depth;
    kinect.enableDepth(depth);
  } 
  else if (key == 'r') {
    rgb = !rgb;
    kinect.enableRGB(rgb);
  }
}

void stop() {
  kinect.quit();
  super.stop();
}


