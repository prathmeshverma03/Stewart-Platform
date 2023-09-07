/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
import processing.serial.*;
import java.util.ArrayList;
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
String PORTNAME = "COM12";           // Port attached to arduino
int BAUD_RATE = 115200;
Float MIN=-3.0;
Float MEAN=0.0;
Float MAX=3.0;
Float MINA=-12.0;
Float MAXA=12.0;
Float MEANA=0.0;

String CSVFILE_NAME = "data.csv";    // Name of csv file
Float INCREMENT=1.0;
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
Table table; 
Serial myPort;  
ArrayList<Float> arr1= new ArrayList<>();
ArrayList<Float> arr2= new ArrayList<>();
ArrayList<Float> arr3= new ArrayList<>();
ArrayList<Float> arr4= new ArrayList<>();
ArrayList<Float> arr5= new ArrayList<>();
ArrayList<Float> arr6= new ArrayList<>();
ArrayList<Float> curr= new ArrayList<>();
int sz,idx,ci;
float x,P;
Float C=180/PI;
Float X,Y,Z,XA,YA,ZA;
boolean isButtonPressed;
boolean isSwitchPressed;
boolean isAnglePressed;
boolean flag1;
boolean flag2;
boolean flag3;
boolean prev;
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
void setup() 
{
  loadData();
  fullScreen(P3D);
  background(0);
  myPort = new Serial(this, PORTNAME, BAUD_RATE);
  myPort.bufferUntil('\n');
  initialize();
}
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
void draw() {
   checkX();
   store();
   if(ifButtonClick()) flip1();
   if(ifSwitchClick()) flip2();
   displayButton();
   displaySwitch();
   displayCoordinates();
   if(!isSwitchPressed)  {
     if(flag1==false) {flag1=true;  flag2=false; background(0); x=50;}
     displayAngle();
     if(ifAngleClick()) flip3();
     if(isAnglePressed!=prev) {background(0); x=50;} 
     if(!isAnglePressed) {drawGraph(); x+=INCREMENT;}
     else {drawAngle(); x+=INCREMENT/2;}
     prev=isAnglePressed;
     
   }
   else { 
     if(flag2==false) {background(0); flag2=true; flag1=false; }
     displayStewart(); 
   }
}
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
void serialEvent(Serial myPort){
    String temp=myPort.readStringUntil('\n');
    if(temp!=null){  
        temp=trim(temp);
        if(temp.equals("done"))  { if(isButtonPressed) sendData(); }
        else {  //curr.set(ci,float(temp));   ci=(ci+1)%6; 
            curr.set(temp.charAt(temp.length()-1)-'A',float(temp.substring(0,temp.length()-1)));
        }
    }
}
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
void loadData() {
  table = loadTable(CSVFILE_NAME, "header");
  arr1.clear(); arr2.clear(); arr3.clear(); arr4.clear(); arr5.clear(); arr6.clear();
  for (TableRow row : table.rows()) {  float x = row.getFloat(0);   arr1.add(x); }
  for (TableRow row : table.rows()) {  float x = row.getFloat(1);   arr2.add(x); }
  for (TableRow row : table.rows()) {  float x = row.getFloat(2);   arr3.add(x); }
  for (TableRow row : table.rows()) {  float x = row.getFloat(3);   arr4.add(x); }
  for (TableRow row : table.rows()) {  float x = row.getFloat(4);   arr5.add(x); }
  for (TableRow row : table.rows()) {  float x = row.getFloat(5);   arr6.add(x); }
}
/**********************************************************************************************************************************************************************/
void initialize(){
    sz=arr1.size();
    idx=0;
    x=50;
    ci=0;
    P=90;
    for(int i=0;i<6;i++) curr.add(0.0);
    isButtonPressed=false;
    isSwitchPressed=false;
    isAnglePressed=false;
    flag1=true;
    flag2=false;
    flag3=false;
    prev=false;
    X=0.0;
    Y=0.0;
    Z=0.0;
    XA=0.0;
    YA=0.0;
    ZA=0.0;
}
/**********************************************************************************************************************************************************************/
void checkX(){
    if(x>=width-15) { background(0); x=50; }
}
void store(){
    X=curr.get(0);
    Y=curr.get(1);
    Z=curr.get(2);
    XA=curr.get(3);
    YA=curr.get(4);
    ZA=curr.get(5); 
}
/**********************************************************************************************************************************************************************/
boolean ifButtonClick(){
    return mousePressed==true && abs(mouseX-100)<=60 && abs(mouseY-60)<=100;
}
/**********************************************************************************************************************************************************************/
void flip1(){
     mousePressed=false;
     if(isButtonPressed==false ) {  isButtonPressed=true;  sendData();}
     else isButtonPressed=false;
}

/**********************************************************************************************************************************************************************/
void displayButton(){
    if(isButtonPressed)   fill(0,255,0);
    else fill(255,0,0);
    ellipse(100,100,60,60);
    stroke(255,255,255);
    fill(255);
    textSize(16);
    if(isButtonPressed)  text("stop", 85,100);
    else text("start", 85,100);
}
/**********************************************************************************************************************************************************************/
void sendData(){
     String s1=Float.toString(arr1.get(idx));
     myPort.write(s1+'@');
     String s2=Float.toString(arr2.get(idx));
     myPort.write(s2+'@');
     String s3=Float.toString(arr3.get(idx));
     myPort.write(s3+'@');
     String s4=Float.toString(arr4.get(idx));
     myPort.write(s4+'@');
     String s5=Float.toString(arr5.get(idx));
     myPort.write(s5+'@');
     String s6=Float.toString(arr6.get(idx));
     myPort.write(s6+'@');
     idx=(idx+1)%sz;
}
/**********************************************************************************************************************************************************************/
void displayX(){
    stroke(255);
    fill(0);
    rect(width/2.0-94,height-60,80,20);
    fill(255,0,0);
    text( "  X = "+X, width/2.0-94,height-46);
}
/**********************************************************************************************************************************************************************/
void displayY(){
    stroke(255);
    fill(0);
    rect(width/2.0-4,height-60,80,20);
    fill(0,255,0);
    text( "  Y = "+Y, width/2.0-4,height-46);
}
/**********************************************************************************************************************************************************************/
void displayZ(){
    stroke(255);
    fill(0);
    rect(width/2.0+94,height-60,80,20);
    fill(0,0,255);
    text( "  Z = "+Z, width/2.0+94,height-46);
}
/**********************************************************************************************************************************************************************/
void displayXangle(){
    stroke(255);
    fill(0);
    rect(width/2.0-94,height-36,80,20);
    fill(255,0,0);
    text( "  X(rot) = "+round(XA*C), width/2.0-94,height-22);
}
/**********************************************************************************************************************************************************************/
void displayYangle(){
    stroke(255);
    fill(0);
    rect(width/2.0-4,height-36,80,20);
    fill(0,255,0);
    text( "  Y(rot) = "+round(YA*C), width/2.0-4,height-22);
}
/**********************************************************************************************************************************************************************/
void displayZangle(){
    stroke(255);
    fill(0);
    rect(width/2.0+94,height-36,80,20);
    fill(0,0,255);
    text( "  Z(rot) = "+round(ZA*C), width/2.0+94,height-22);
}
/**********************************************************************************************************************************************************************/
void displayCoordinates(){
     displayX();
     displayY();
     displayZ();
     displayXangle();
     displayYangle();
     displayZangle();
}
/**********************************************************************************************************************************************************************/
void drawGraphX(){
    fill(255,0,0);
    stroke(240,0,0);
    ellipse(x,height-((X-MIN)*height)/(MAX-MIN),10,10);
}
/**********************************************************************************************************************************************************************/
void drawGraphY(){
    fill(0,255,0);
    stroke(0,240,0);
    ellipse(x,height-((Y-MIN)*height)/(MAX-MIN),10,10);
}
/**********************************************************************************************************************************************************************/
void drawGraphZ(){
    fill(0,0,255);
    stroke(0,0,240);
    ellipse(x,height-((Z-MIN)*height)/(MAX-MIN),10,10);
}
/**********************************************************************************************************************************************************************/
void drawGraph(){
    if(  Float.compare(X,Y)==0 && Float.compare(Y,Z)==0){
       fill(255,255,255);
       stroke(240,240,240);
       ellipse(x,height-((X-MIN)*height)/(MAX-MIN),10,10);
    }
    else if(Float.compare(X,Y)==0){
       fill(255,255,0);
       stroke(240,240,0);
       ellipse(x,height-((X-MIN)*height)/(MAX-MIN),10,10);
       drawGraphZ();
    }
    else if(Float.compare(X,Z)==0){
       fill(255,0,255);
       stroke(240,0,240);
       ellipse(x,height-((X-MIN)*height)/(MAX-MIN),10,10);
       drawGraphY();
    }
    else if(Float.compare(Z,Y)==0){
       fill(0,255,255);
       stroke(0,240,240);
       ellipse(x,height-((Y-MIN)*height)/(MAX-MIN),10,10);
       drawGraphX();
    }
    else {
      drawGraphX();
      drawGraphY();
      drawGraphZ();
    }  
}
/**********************************************************************************************************************************************************************/
void drawAngleX(){
    fill(255,0,0);
    stroke(240,0,0);
    ellipse(x,height-((XA-MINA)*height)/(MAXA-MINA),10,10);
}
/**********************************************************************************************************************************************************************/
void drawAngleY(){
    fill(0,255,0);
    stroke(0,240,0);
    ellipse(x,height-((YA*C-MINA)*height)/(MAXA-MINA),10,10);
}
/**********************************************************************************************************************************************************************/
void drawAngleZ(){
    fill(0,0,255);
    stroke(0,0,240);
    ellipse(x,height-((ZA*C-MINA)*height)/(MAXA-MINA),10,10);
}
/**********************************************************************************************************************************************************************/
void drawAngle(){
    if(  Float.compare(XA,YA)==0 && Float.compare(YA,ZA)==0){
       fill(255,255,255);
       stroke(240,240,240);
       ellipse(x,height-((XA*C-MINA)*height)/(MAXA-MINA),10,10);
    }
    else if(Float.compare(XA,YA)==0){
       fill(255,255,0);
       stroke(240,240,0);
       ellipse(x,height-((XA*C-MINA)*height)/(MAXA-MINA),10,10);
       drawAngleZ();
    }
    else if(Float.compare(XA,ZA)==0){
       fill(255,0,255);
       stroke(240,0,240);
       ellipse(x,height-((XA*C-MINA)*height)/(MAXA-MINA),10,10);
       drawAngleY();
    }
    else if(Float.compare(ZA,YA)==0){
       fill(0,255,255);
       stroke(0,240,240);
       ellipse(x,height-((YA*C-MINA)*height)/(MAXA-MINA),10,10);
       drawAngleX();
    }
    else {
      drawAngleX();
      drawAngleY();
      drawAngleZ();
    }  
}
/**********************************************************************************************************************************************************************/
void displayAngle(){
    if(isAnglePressed)   fill(0,255,0);
    else fill(255,0,0);
    ellipse(100,200,60,60);
    stroke(255,255,255);
    fill(255);
    textSize(16);
    if(isAnglePressed)  text("X,Y,Z", 85,200);
    else text("Angle", 85,200);
}
/**********************************************************************************************************************************************************************/
boolean ifAngleClick(){
    return mousePressed==true && abs(mouseX-100)<=60 && abs(mouseY-200)<=100;
}
/**********************************************************************************************************************************************************************/
void flip3(){
     mousePressed=false;
     if(isAnglePressed==false ) {  isAnglePressed=true;  }
     else isAnglePressed=false;
}
/**********************************************************************************************************************************************************************/
boolean ifSwitchClick(){
    return mousePressed==true && abs(mouseX-200)<=60 && abs(mouseY-60)<=100;
}
/**********************************************************************************************************************************************************************/
void displaySwitch(){
    if(isSwitchPressed)   fill(0,255,0);
    else fill(255,0,0);
    ellipse(200,100,60,60);
    stroke(255,255,255);
    fill(255);
    textSize(16);
    if(isSwitchPressed)  text("graph", 185,100);
    else text("model", 185,100);
}

/**********************************************************************************************************************************************************************/
void flip2(){
     mousePressed=false;
     if(isSwitchPressed==false ) {  isSwitchPressed=true;  }
     else isSwitchPressed=false;
}
/**********************************************************************************************************************************************************************/
void displayStewart(){
  setOrientation();
  drawBackground();
  drawPlatform();
  drawBase();
  drawRods();
 
}
/**********************************************************************************************************************************************************************/
void setOrientation(){
    translate(width/2, height/2, 0); 
    rotateX(PI/4);
    rotateZ(0);
    rotateY(0);
}
/**********************************************************************************************************************************************************************/
void drawBackground(){
     stroke(0,0,0);
     fill(0,0,0);
     beginShape();
       vertex(-234,-200,120);
       vertex( 234,-200,120);
       vertex( 400,-200,-600);
       vertex(-400,-200,-600);
    endShape();
}
/**********************************************************************************************************************************************************************/
void drawPlatform(){
    fill(0,0,255);
    stroke(0,0,255);
    pushMatrix();
      beginShape();
        rotateZ(-ZA);
        rotateY(YA);
        rotateX(XA);
        vertex(-86+X*10, 0-Y*10,P+Z*10);
        vertex( -42+X*10,  75-Y*10,P+Z*10);
        vertex(   42+X*10,  75-Y*10,P+Z*10);
        vertex(86+X*10, 0-Y*10,P+Z*10);
        vertex( 42+X*10, -75-Y*10,P+Z*10);
        vertex(   -42+X*10,-75-Y*10,P+Z*10);
      endShape();
    popMatrix();
}
/**********************************************************************************************************************************************************************/
void drawBase(){
     beginShape();
      vertex(-86, 0,-P);
      vertex( -42, 75,-P);
      vertex(   42,75,-P);
      vertex(86,0,-P);
      vertex( 42,-75,-P);
      vertex(   -42,-75,-P);
    endShape();
      
    beginShape();
      vertex(-86, 0,-P-50);
      vertex( -42, 75,-P-50);
      vertex(   42,75,-P-50);
      vertex(86,0,-P-50);
      vertex( 42,-75,-P-50);
      vertex(   -42,-75,-P-50);
    endShape();
      
}
/**********************************************************************************************************************************************************************/
void drawRods(){
     stroke(192,192,192);
     float d=200;
     float div=(2*P+Z*10)/(d+1);
     float st=0.0;
     float end=div;
     for(int i=0;i<=d;i++){
          pushMatrix();
            float c=i/d;
            rotateZ(-ZA*c);
            rotateY(YA*c);
            rotateX(XA*c);
            drawLine(st,end,c);
            st+=div;
            end+=div;
          popMatrix();
      }
}
/**********************************************************************************************************************************************************************/
void drawLine(float z1,float z2,float c){
    strokeWeight(2);
    line(-86+X*10*c, 0-Y*10*c,-P+z1,-86+X*10*c, 0-Y*10*c,-P+z2);
    line(-42+X*10*c, 75-Y*10*c,-P+z1,-42+X*10*c,  75-Y*10*c,-P+z2);
    line( 42+X*10*c,75-Y*10*c,-P+z1, 42+X*10*c,  75-Y*10*c,-P+z2);
    line(86+X*10*c,0-Y*10*c,-P+z1,86+X*10*c, 0-Y*10*c,-P+z2);
    line(42+X*10*c,-75-Y*10*c,-P+z1,42+X*10*c, -75-Y*10*c,-P+z2);
    line( -42+X*10*c,-75-Y*10*c,-P+z1,-42+X*10*c,-75-Y*10*c,-P+z2);

}

/**********************************************************************************************************************************************************************/
