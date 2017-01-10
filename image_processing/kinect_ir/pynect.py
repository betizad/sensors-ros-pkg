import SimpleOpenNI.*;
SimpleOpenNI  context;
void setup(){
  size(640 * 2 + 10, 480);
  context = new SimpleOpenNI(this);
  if(context.isInit() == false){
     println("fail"); 
     exit();
     return;
  }
  context.enableDepth();

  // enable ir generation
  //context.enableIR(); old line 
  context.enableIR(1,1,1); //new line

  background(200,0,0);
}

void draw(){
  context.update();
  image(context.depthImage(),context.depthWidth() + 10,0);

  image(context.irImage(),0,0);
}
