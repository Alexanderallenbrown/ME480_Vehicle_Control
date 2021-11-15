var kcar;
var croad;

var t;
var dt;
var oldt;

var pcontroller;

var stepB;
var stepS;

var uS;

var kpS;
var LS;

var Im_touched = false;


function setup() {
  t = millis();
  oldt = 0;
  cnv = createCanvas(750, 500);
  kcar = new kinCar(55, 1.5);
  //bmCar_nonlin(U,m,a,b,I)
  //ftire = new PacejkaTire(2000,1.0,1.0);
  //rtire = new PacejkaTire(2000,1.0,1.0);
  nlcar = new bmcar_nonlin(5, 2000, 0.5, 1, 5000); //nonlinear car
  croad = new carRoad(100, 15, 5, 50, 350);
  pcontroller = new prevCont(.5, 1, 0);


  //XSlider(ixorg, iyorg, ilen, imin, imax, islpos, ilabel)
  stepS = new XSlider(50, 50, 100, -1, 1, 1, 'lane step (m)');
  uS = new XSlider(50, 80, 100, 1, 20, 5, 'speed (m/s)');
  prevS = new XSlider(50, 110, 100, 0.01, 5, 5, 'lookahead (m)');
  kpS = new XSlider(50, 140, 100, 0.001, .25, .05, 'Kp');
  kdS = new XSlider(50,170,100,0,1,0,'100*Kd');
  steerzS = new XSlider(400, 110, 100, 0.1, 1, .7, 'Steer Motor zeta');
  steerwS = new XSlider(400, 140, 100, PI, 10 * PI, PI, 'Steer Motor omega');

  //
  //choose stepped road or not
  stepB = new RadioButton(200, 50, 20, "step active");
  actuatorB = new RadioButton(200, 100, 20, "actuator dynamics");
  vehicleB = new RadioButton(200, 150, 20, "vehicle dynamics");

  stepS.drawSlider();
  uS.drawSlider();
  stepB.updateRadio();


  //steering wheel
  wheel = new steerWheel(400, 50, 50, 20);
  //steering motor model
  steermotor = new steerMotor(.7, PI);



}


function draw() {
  //print(ftire.calcFy(1000,0.01,0,0));
  background(255);
  stroke(0);
  t = millis() / 1000.0;
  dt = t - oldt;
  oldt = t;

  var roadpos = stepS.slpos;
  if (!stepB.state) {
    roadpos = 0;
  }
  var U = uS.slpos;

  //update gains if needed and preview
  pcontroller.kp = kpS.slpos;
  pcontroller.kd = kdS.slpos/100;
  pcontroller.prevLength = prevS.slpos;
  if (vehicleB.state) {
    delta_r = pcontroller.steer(nlcar, croad, dt);
  } else {
    delta_r = pcontroller.steer(kcar, croad, dt);
  }

  //update steermotor
  steermotor.z = steerzS.slpos;
  steermotor.w = steerwS.slpos;
  if (actuatorB.state) {
    delta = steermotor.update(delta_r, dt);
  } else {
    delta = delta_r;
  }
  if (vehicleB.state) {
    nlcar.update(U, delta, dt);
    kcar.x = nlcar.x;
    kcar.y = nlcar.y;
    kcar.psi = nlcar.psi;
  }
  else{
    kcar.update(U,delta,dt);
    
    nlcar.x = kcar.x;
    nlcar.y = kcar.y;
    nlcar.V = 0;
    nlcar.psidot =0;
    nlcar.psi = kcar.psi;
  }
  //print(nlcar.y);
  //print(uS.slpos);
  //kcar.psi = 0.1
  //kcar.delta = 0.1
  //draw road

  croad.drawRoad(roadpos);
  //draw car
  if (vehicleB.state) {
    croad.drawCar(nlcar, dt);
    pcontroller.drawPoints(nlcar, croad, dt);
  } else {
    croad.drawCar(kcar, dt);
    pcontroller.drawPoints(kcar, croad, dt);
  }


  //draw interface
  strokeWeight(1);
  stepS.drawSlider();
  uS.drawSlider();
  prevS.drawSlider();
  kpS.drawSlider();
  kdS.drawSlider();
  if(actuatorB.state){
  steerzS.drawSlider();
  steerwS.drawSlider();
}

  stepB.updateRadio();
  actuatorB.updateRadio();
  vehicleB.updateRadio();
  if (vehicleB.state) {
    vehicleB.label = "nonlinear vehicle";
  } else {
    vehicleB.label = "kinematic vehicle";
  }

  //wheel
  wheel.drawWheel(delta_r, delta);

}

this.prevCont = function(prevLength, kp, kd) {
  this.prevLength = prevLength;
  this.kp = kp;
  this.kd = kd;
  this.e = 0;
  this.olde = 0;
  this.edot = 0;
  this.yp = 0;
  this.xp = 0;
  this.delta = 0;

  this.calce = function(carmodel, roadmodel, dt) {
    this.yp = carmodel.y + this.prevLength * sin(carmodel.psi);
    this.xp = carmodel.x + this.prevLength * cos(carmodel.psi);
    this.e = roadmodel.roady - this.yp;
    this.edot = (this.e - this.olde) / dt;
    this.olde = this.e;
  }
  this.steer = function(carmodel, roadmodel, dt) {
    this.calce(carmodel, roadmodel, dt);
    this.delta = this.kp * this.e + this.kd * this.edot;
    return this.delta;
  }
  this.drawPoints = function(carmodel, roadmodel, dt) {
    translate(roadmodel.xCenter, roadmodel.yCenter);
    scale(roadmodel.scale);

    translate(0, -carmodel.y);
    rotate(-carmodel.psi);
    strokeWeight(1 / roadmodel.scale);
    stroke(color(255, 0, 0));
    line(0, 0, this.prevLength, 0);
    translate(this.prevLength, 0);
    rotate(carmodel.psi);
    fill(color(0, 255, 0));
    stroke(color(0, 255, 0));
    line(0, 0, 0, -this.e);
    ellipse(0, -this.e, .1, .1, -1);

    //now undo the transforms
    resetMatrix();
  }
}

function carRoad(roadscale, xWindow, yWindow, xCenter, yCenter) {
  this.scale = roadscale; //converts from meters to PIXELS
  this.xWindow = xWindow; //road window in METERS... "radius"
  this.yWindow = yWindow;
  this.xCenter = xCenter; //center point in PIXELS
  this.yCenter = yCenter;
  this.nspokes = 6;
  this.wheel_theta = 0;
  this.wheel_omega = 0;
  this.spokex = [];
  this.spokey = [];
  this.tire_length = 0.6; //diameter of tire in METERS
  this.roady = 0;
  for (k = 0; k < this.nspokes; k++) {
    this.spokex.push((this.tire_length) / 2 * cos(2 * PI / this.nspokes * k + this.wheel_theta)); //initialize spoke x locations
    this.spokey.push((this.tire_length) / 2 * sin(2 * PI / this.nspokes * k + this.wheel_theta)); //initialize spoke y locations
  }
  //print(this.spokex);



  this.drawRoad = function(roadY) {
    //roadY comes in in METERS
    this.roady = roadY;
    translate(this.xCenter, this.yCenter);
    scale(this.scale);
    stroke(0);
    strokeWeight(1 / this.scale);
    line(-this.xWindow, -roadY, this.xWindow, -roadY);
    resetMatrix();
  }

  this.drawCar = function(carmodel, dt) {
    //print(this.xCenter);
    translate(this.xCenter, this.yCenter);
    scale(this.scale);

    translate(0, -carmodel.y);
    rotate(-carmodel.psi);
    strokeWeight(5 / this.scale);
    stroke(0);
    line(0, 0, carmodel.L, 0);

    strokeWeight(1 / this.scale);
    //now we work in "meters" to draw the car. wheels first.
    this.wheel_omega = -carmodel.U / (this.tire_length / 2);
    this.wheel_theta += this.wheel_omega * dt
    //this.wheel_theta =this.wheel_theta+ dt * this.wheel_omega;
    for (k = 0; k < this.nspokes; k++) {
      //print((this.tire_length) / 2 * cos(2 * PI / this.nspokes * k + this.wheel_theta));
      this.spokex[k] = (this.tire_length) / 2 * cos(2 * PI / this.nspokes * k + this.wheel_theta);
      this.spokey[k] = (this.tire_length) / 2 * sin(2 * PI / this.nspokes * k + this.wheel_theta); //initialize spoke y locations
    }
    //print(this.wheel_theta);


    //real wheel top view
    stroke(0);
    fill(150);
    rectMode(CENTER);
    rect(0, 0, this.tire_length, this.tire_length * 0.2); //draw box for rear tire
    //now draw the spokes on the wheel
    for (k = 0; k < this.nspokes; k++) {
      if (this.spokey[k] < 0) {
        stroke(128);
      } else {
        stroke(0);
      }
      strokeWeight(1 / this.scale);
      line(this.spokex[k], -0.1 * this.tire_length, this.spokex[k], 0.1 * this.tire_length);
    }
    //front wheel
    //front wheel top view
    translate(carmodel.L, 0);
    rotate(-carmodel.delta);
    stroke(0);
    rect(0, 0, this.tire_length, 0.2 * this.tire_length);
    //now draw the spokes on the wheel
    for (k = 0; k < this.nspokes; k++) {
      if (this.spokey[k] < 0) {
        stroke(128);
      } else {
        stroke(0);
      }
      line(this.spokex[k], -0.1 * this.tire_length, this.spokex[k], 0.1 * this.tire_length);
      //print(this.spokex)
    }
    //undo all of our transforms
    resetMatrix();

    //end drawCar
  }

  //end carRoad
}

function steerWheel(x, y, size, ratio) {
  this.x = x;
  this.y = y;
  this.size = size;
  this.ratio = ratio;

  this.drawWheel = function(delta_r, delta) {
    //actual wheel angle
    translate(x, y);
    rotate(-delta * this.ratio);
    stroke(0);
    fill(255);
    strokeWeight(5);
    ellipse(0, 0, size, size, 1);
    line(0, 0, size / 2 * cos(-PI / 2), size / 2 * sin(-PI / 2));
    line(0, 0, size / 2 * cos(-PI / 2 + 2 * PI / 3), size / 2 * sin(-PI / 2 + 2 * PI / 3));
    line(0, 0, size / 2 * cos(-PI / 2 + 4 * PI / 3), size / 2 * sin(-PI / 2 + 4 * PI / 3));
    resetMatrix();
    //desired wheel angle
    translate(x, y);
    rotate(-delta_r * this.ratio);
    stroke(color(255, 0, 0));
    noFill();
    strokeWeight(2);
    ellipse(0, 0, size, size, 1);
    line(0, 0, size / 2 * cos(-PI / 2), size / 2 * sin(-PI / 2));
    line(0, 0, size / 2 * cos(-PI / 2 + 2 * PI / 3), size / 2 * sin(-PI / 2 + 2 * PI / 3));
    line(0, 0, size / 2 * cos(-PI / 2 + 4 * PI / 3), size / 2 * sin(-PI / 2 + 4 * PI / 3));
    resetMatrix();
  }

}


function steerMotor(z, w) {
  //approximates steer motor dynamics as (2*z*wn*s+wn^2)/(s^2+2*z*wn*s+wn^2)
  this.z = z
  this.w = w;
  this.delta_r = 0;
  this.delta_rdot = 0;
  this.delta_rold = 0;
  this.e = 0;
  this.e_old = 0;
  this.delta = 0;
  this.deltadot = 0;
  this.deltaold = 0;

  this.statederivs = function(delta, deltadot, delta_r, delta_rdot) {
    deltaddot = 2 * this.z * this.w * (delta_rdot - deltadot) + this.w * this.w * (delta_r - delta);
    return [deltadot, deltaddot];
  }
  this.update = function(delta_r, dt) {
    this.delta_r = delta_r;
    this.delta_rdot = (this.delta_r - this.delta_rold) / dt;
    this.deltadot = (this.delta - this.deltaold) / dt;
    this.deltaold = this.delta;
    this.delta_rold = this.delta_r;
    //this uses Heun's method (trapezoidal)
    xdot1 = this.statederivs(this.delta, this.deltadot, this.delta_r, this.delta_rdot);
    //first calculation
    deltaprime = this.delta + xdot1[0] * dt;
    deltadotprime = this.deltadot + xdot1[1] * dt;
    //now compute again
    xdot2 = this.statederivs(deltaprime, deltadotprime, this.delta_r, this.delta_rdot);
    //now compute the final update
    this.delta = this.delta + dt / 2 * (xdot1[0] + xdot2[0]);
    this.deltadot = this.deltadot + dt / 2 * (xdot1[1] + xdot2[1]);
    return this.delta;

  }

}



function kinCar(U, L) {
  this.L = L
  this.U = U;
  this.x = 0;
  this.y = 0;
  this.psi = 0;
  this.delta = 0;

  this.statederivs = function(delta, x, y, psi) {
    psidot = this.U * delta / this.L;
    xdot = this.U * cos(psi);
    ydot = this.U * sin(psi);
    return [xdot, ydot, psidot];
  }
  this.update = function(U, delta, dt) {
    this.delta = delta;
    //this uses Heun's method (trapezoidal)
    this.U = U;
    xdot1 = this.statederivs(delta, this.x, this.y, this.psi);
    //first calculation
    xprime = this.x + xdot1[0] * dt;
    yprime = this.y + xdot1[1] * dt;
    psiprime = this.psi + xdot1[2] * dt;
    //now compute again
    xdot2 = this.statederivs(delta, xprime, yprime, psiprime);
    //now compute the final update
    this.x = this.x + dt / 2 * (xdot1[0] + xdot2[0]);
    this.y = this.y + dt / 2 * (xdot1[1] + xdot2[1]);
    this.psi = this.psi + dt / 2 * (xdot1[2] + xdot2[2]);
  }

}

function XSlider(ixorg, iyorg, ilen, imin, imax, islpos, ilabel) {
  this.xorg = ixorg;
  this.yorg = iyorg;
  this.len = ilen;
  this.slpos = islpos;
  this.min = imin;
  this.max = imax;
  this.label = ilabel;
  this.sliderstroke = color(0, 0, 0);
  this.sliderfill = color(0, 0, 0);
  this.was_pressed = false;
  this.held = false;

  this.drawSlider = function() {
    //box_x = (slpos-min)*len/(max-min);
    this.updateSlider();
    stroke(this.sliderstroke);
    fill(this.sliderfill);
    line(this.xorg, this.yorg, this.xorg + this.len, this.yorg);
    rectMode(CENTER);
    this.box_x = (this.slpos - this.min) * this.len / (this.max - this.min);
    rect(this.xorg + this.box_x, this.yorg, this.len * .2, this.len * .1);
    textSize(12);
    text(this.label + ": " + nf(this.slpos, 1, 2), this.xorg, this.yorg - this.len * .1);
  }

  this.updateSlider = function() {
    this.box_x = (this.slpos - this.min) * this.len / (this.max - this.min);
    //console.log(touches)
    if ((Im_touched && !this.was_pressed && !this.held) || (mouseIsPressed && !this.was_pressed && !this.held)) {

      //println((xorg+box_x-mouseX));
      if (abs(this.xorg + this.box_x - mouseX) < 0.2 * this.len && abs(this.yorg - mouseY) < 0.2 * this.len) {
        this.held = true;
        //println("held");
      }
    } else if ((Im_touched && this.held) || (Im_touched && this.held)) {
      this.held = true;
    } else {
      this.held = false;
    }
    //println(mouseX-xorg);
    this.was_pressed = Im_touched || mouseIsPressed;
    if (this.held == true) {
      //update slider if the box is being dragged
      this.box_x = mouseX - this.xorg;
      if (mouseX > this.xorg + this.len) {
        this.box_x = this.len;
      }
      if (mouseX < this.xorg) {
        this.box_x = 0;
      }
      this.slpos = this.min + (this.max - this.min) * this.box_x / this.len;
    }
  }
}


function RadioButton(ix, iy, id, ilabel) {
  this.x = ix;
  this.y = iy;
  this.d = id;
  this.state = false;
  this.touched = false;
  this.wastouched = false;
  this.newtouch = false;
  this.label = ilabel;

  this.updateRadio = function() {
    //detect whether the radio button is 
    if (mouseIsPressed == true) {
      if (sqrt(pow(mouseX - this.x, 2) + pow(mouseY - this.y, 2)) <= this.d / 2) {
        this.touched = true;
      }
    } else {
      this.touched = false;
    }


    if (this.touched == true & this.wastouched == false) {
      this.state = !this.state;
    }
    //update value of touched
    this.wastouched = this.touched;

    //draw the button
    this.drawRadio();
  }


  this.drawRadio = function() {
    if (this.state == false) {
      fill(255);
    } else {
      fill(0);
    }
    stroke(0);
    //draw the radio button
    ellipse(this.x, this.y, this.d, this.d, 2);
    fill(0);
    stroke(0);
    textSize(12);
    text(this.label, this.x + this.d, this.y);
  }

}

function touchStarted() {
  Im_touched = true;
}

function touchEnded() {
  Im_touched = false;
}

function windowResized() {
  centerCanvas();
}

function centerCanvas() {
  var x = (windowWidth - width) / 2;
  var y = (windowHeight - height) / 2;
  cnv.position = [x, y];
}



function PacejkaTire(iFz, ifric_x_max, ifric_y_max) {
  //parameters explained: http://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/
  this.fyparms = [-22.1, 1011, 1078, 1.82, .208, -0.00, -0.354, .707, .028, 0.0, 14.8, .022, 0.0];
  this.mzparms = [-2.72, -2.28, -1.86, -2.73, 0.110, -0.07, .643, -4.04, 0.015, -0.066, .945, 0.03, 0.07];
  this.fxparms = [-21.3, 1144, 49.6, 226, 0.069, -0.006, 0.056, .486];
  //these parameters were taken from: 
  this.K = 50.0;
  this.alpha = 0;
  this.lambda = 0;
  this.camb = 0;
  this.Fz = iFz;
  //this.updateFyparams(this.Fz);
  this.fric_x_max = this.ifric_x_max;
  this.fric_y_max = this.ifric_y_max;

  this.updateFyparams = function(iFz) {
    this.Fz = iFz;
    //use braking friction to cap y friction
    //allowable_acceleration = sqrt((self.fxb*self.g)**2*(1-lat_accel_here**2/((self.g*self.fy)**2))) #this is the magnitude of the acceleration the tires can tolerate here
    this.C = 1.3; //sqrt(pow(fric_y_max,2)*(1-pow(fric_x,2)/pow(fric_x_max,2)));//this is the ultimate friction. uses friction ellipse to scale.
    this.D = this.fyparms[0] * pow(this.Fz * 0.001, 2) + this.fyparms[1] * this.Fz * 0.001;
    this.C_alpha = this.fyparms[2] * sin(this.fyparms[3] * atan(this.fyparms[4] * this.Fz));
    this.B = this.C_alpha / (this.C * this.D);
    this.E = this.fyparms[5] * pow(this.Fz * 0.001, 2) + this.fyparms[6] * this.Fz * 0.001 + this.fyparms[7];
    this.Sh = -this.fyparms[8] * this.camb * PI / 180;
    this.Sv = this.camb * (-this.fyparms[9] * PI / 180 * pow(this.Fz * 0.001, 2) + -this.fyparms[10] * PI / 180 * this.Fz * 0.001);
    this.delB = -this.fyparms[11] * abs(this.camb) * this.B * PI / 180;
    this.B = this.B + this.delB;
  }

  this.calcFy = function(iFz, ialpha, ilambda, icamb) {
    this.lambda = ilambda;
    this.Fz = iFz;
    this.alpha = ialpha;
    this.alpha_deg = this.alpha * 180 / PI;
    this.camb = icamb;
    this.updateFyparams(this.Fz);
    this.phi = (1 - this.E) * (this.alpha_deg + this.Sh) + this.E / (this.B) * atan(this.B * (this.alpha_deg + this.Sh));
    this.Fy = this.D * sin(this.C * atan(this.B * this.phi)) + this.Sv;
    //now we adjust for longitudinal slip (per adams pacejka2002 manual)
    //we will use friction ellipse instead of "cosine weighting functions"
    if (0) { //abs(this.lambda)>0 || abs(this.alpha)>0 || abs(this.camb)>0) {
      //this is not part of Pacejka's original formulation, but was developed by MSC
      var kappac = this.lambda;
      var alphac = this.alpha + this.Sh + this.Sv / this.K; //NOTE: THESE K /100 are just made up!! fix?
      var alphastar = sin(alphac);
      var beta = acos(abs(kappac) / sqrt(pow(kappac, 2) + pow(alphastar, 2))); //NOTE had abs(kappac)
      var muy_now = (this.Fy - this.Sv) / this.Fz;
      var mux_max = this.Dx / this.Fz;
      var muy = abs(tan(beta) / sqrt(pow(1 / mux_max, 2) + pow(tan(beta) / (muy_now + 0.01), 2)));
      this.Fy_comb = this.Fy * muy / abs(0.001 + muy_now);
    } else {
      this.Fy_comb = this.Fy;
    }
    return -this.Fy_comb; //combined case cornering force
  }

  this.updateFxparams = function(iFz) {
    this.Fz = iFz;
    this.Cx = 1.65; //fric_x_max;//we scale the lat accel, so leave this alone??
    this.Dx = this.fxparms[0] * pow(this.Fz * 0.001, 2) + this.fxparms[1] * this.Fz * 0.001;
    this.BCDx = (this.fxparms[2] * pow(this.Fz * 0.001, 2) + this.fxparms[3] * this.Fz * 0.001) / (exp(this.fxparms[4] * 0.001 * this.Fz)); //
    //println("BCDx= "+exp(-fxparms[4]*Fz));
    this.Bx = this.BCDx / (this.Cx * this.Dx);
    //E = fyparms[5]*pow(Fz*.001, 2)+fyparms[6]*Fz*.001+fyparms[7];
    this.Ex = this.fxparms[5] * pow(this.Fz * 0.001, 2) + this.fxparms[6] * this.Fz * 0.001 + this.fxparms[7];
  }

  this.calcFx = function(iFz, ilambda, ialpha) {
    this.alpha = ialpha;
    this.Fz = iFz;
    this.lambda = ilambda;
    this.updateFxparams(this.Fz);
    this.updateFyparams(this.Fz);
    this.phix = (1 - this.Ex) * this.lambda * 100 + this.Ex / (this.Bx + 0.001) * atan(this.Bx * this.lambda * 100);
    //println("Bx, Phix= "+str(Bx)+","+str((1-Ex)*lambda*100));
    this.Fx = this.Dx * sin(this.Cx * atan(this.Bx * this.phix));
    if (abs(this.alpha) > 0 || abs(this.lambda) > 0 || abs(this.camb) > 0) {
      //this is not part of Pacejka's original formulation, but was developed by MSC
      var kappac = this.lambda;
      var alphac = this.alpha + this.Sh + this.Sv / this.K;
      var alphastar = sin(alphac);
      var beta = acos(abs(kappac) / sqrt(pow(kappac, 2) + pow(alphastar, 2))); //NOTE had abs(kappac) in num
      var mux_now = (this.Fx) / this.Fz; //
      var muy_max = this.D / this.Fz;
      var mux = 1 / sqrt(pow(1 / (mux_now + 0.01), 2) + pow(tan(beta) / muy_max, 2));
      this.Fx_comb = this.Fx * mux / abs(mux_now + 0.001);
    } else {
      this.Fx_comb = this.Fx;
    }
    return this.Fx_comb; //combined case cornering force
  }
}

function bmcar_nonlin(U, m, a, b, I) {
  this.L = a + b;
  this.a = a;
  this.b = b;
  this.m = m;
  this.I = I;
  this.U = U;
  this.x = 0;
  this.y = 0;
  this.psi = 0;
  this.psidot = 0;
  this.delta = 0;
  this.g = 9.81;
  this.Fzf = this.m * this.g * this.b / this.L;
  this.Fzr = this.m * this.g * this.a / this.L;
  this.alpha_f = 0;
  this.alpha_r = 0;
  this.V = 0;
  this.ftire = new PacejkaTire(this.Fzf/2, 1.5, 1.5);
  this.rtire = new PacejkaTire(this.Fzr/2, 1.5, 1.5);
  //this.ftire = new PacejkaTire(1000,1.2,1.5);
  //this.rtire = new PacejkaTire(1000,1.2,1.5);
  //this.tire = new PacejkaTire(4000, 1.5, 1.2);//Fz,mux,muy

  this.statederivs = function(delta, x, y, V, psi, psidot) {
    alpha_f = (V + this.a * psidot) / this.U - delta;
    alpha_r = (V - this.b * psidot) / this.U;
    //this.calcFy=function(iFz, ialpha, ilambda, icamb)
    Fyf = this.ftire.calcFy(this.Fzf/2, alpha_f, 0, 0);
    Fyr = this.rtire.calcFy(this.Fzr/2, alpha_r, 0, 0);
    //tire = new PacejkaTire(1000, 1.5, 1.2);//Fz,mux,muy
    //print(this.ftire.calcFy(1000,0.01,0,0));
    //Fyr = this.rtire.calcFy(this.Fzr,alpha_r,0,0);

    //print();
    //print(this.Fzf);

    xdot = this.U * cos(psi) - V * sin(psi);
    ydot = this.U * sin(psi) + V * cos(psi);
    Vdot = 1 / this.m * (2*Fyf + 2*Fyr)-this.U*psidot;
    psiddot = 1 / this.I * (this.a * 2*Fyf - 2*this.b * Fyr);
    return [xdot, ydot, Vdot, psidot, psiddot];
  }
  this.update = function(U, delta, dt) {
    this.delta = delta;
    //this uses Heun's method (trapezoidal)
    this.U = U;
    xdot1 = this.statederivs(delta, this.x, this.y, this.V, this.psi, this.psidot);
    //print();
    //first calculation
    xprime = this.x + xdot1[0] * dt;
    yprime = this.y + xdot1[1] * dt;
    Vprime = this.V + xdot1[2] * dt;
    psiprime = this.psi + xdot1[3] * dt;
    psidotprime = this.psidot + xdot1[4] * dt;
    //now compute again
    xdot2 = this.statederivs(delta, xprime, yprime, Vprime, psiprime, psidotprime);
    //now compute the final update
    this.x = this.x + dt / 2 * (xdot1[0] + xdot2[0]);
    this.y = this.y + dt / 2 * (xdot1[1] + xdot2[1]);
    this.V = this.V + dt / 2 * (xdot1[2] + xdot2[2]);
    this.psi = this.psi + dt / 2 * (xdot1[3] + xdot2[3]);
    this.psidot = this.psidot + dt / 2 * (xdot1[4] + xdot2[4]);
  }

}