/////////////////////////////
//// TREBUCHET SIMULATOR ////
/////////////////////////////

// Author: Ryan O'Sullivan
// Release: A
// Date: 08/29/2016



////////////////////////////////////
// Program Initilization Packages //
////////////////////////////////////


package main

import (
	"math"
	"fmt"
    "os"
    "time"
    "log"
    "encoding/csv"
    "strconv"
)

///////////////////////
// Global Parameters //
///////////////////////

// Gravity
var g float64 = 9.81

// Wind Speed
var Wind_v float64 = 0.0

// Air Density
var ro float64 = 1.225

//////////////////////////
// Trebuchet Parameters //
//////////////////////////

// Length of long arm from pivot to sling attachment point
var L_Arm_Lo float64 = 6.792

// Length of short arm from pivot to weight attachment point
var L_Arm_Sh float64 = 1.75

// Lenght of weight arm
var L_Arm_We float64 = 2.0

// Lenght of sling
var L_Sling float64 = 6.833

// Height of pivot
var H_Pivot float64 = 5.0

// Mass of Counter Weight
var CW_Mass float64 = 550.0

// Inertia of CW_Mass
var CW_I float64 = 1.0

// Mass of Long Arm
var Arm_Mass float64 = 10.0

//Inertial of Long arm
var Arm_I float64 = 65.0

// Lenght from pivot to CG of long arm
var L_Arm_CG float64 = 2.52

// Release angle (radian)
var Rel_Angle float64 = .785398


///////////////////////////
// Projectile Parameters //
///////////////////////////

// Projectile Mass
var Proj_Mass float64 = 0.15

// Projectile Diameter
var Proj_Dia float64 = 0.076

// Drag Coeff
var Cd float64 = 0.47

// Area
var Area = math.Pi/4.0 * math.Pow(Proj_Dia,2)

////////////////////////
// Initial Conditions //
////////////////////////

// Arm initial angle
var Aq_0 float64 = 1.8326 // 105deg

// Arm initial velocity
var DAq_0 float64 = 0.0 // 0 rad/sec

// Weight initial angle
var Wq_0 float64 = 4.97419

// Weight initial velocity
var DWq_0 float64 = 0.0

//Sling initial angle
var Sq_0 float64 = 2.35619

// Sling initial velocity
var DSq_0 float64 = 0.0


/////////////////////
// Main Simulation //
/////////////////////

//Input variables for Y:
// Y = [
// Aq
// Aq'
// Wq
// Wq'
// Sq
// Sq'
// ]

//Arm_Mass, CW_Mass, Proj_Mass, Arm_I, CW_I, L_Arm_Lo, L_Arm_CG, L_Arm_Sh, L_Arm_We, L_Sling

//First Simulation
func treb_1(time float64, Y []float64) []float64 {
  
  ret := make([]float64,len(Y))

  M11 := (-1.0)*Proj_Mass*math.Pow(L_Arm_Lo,2)*(-1+2*math.Sin(Y[0])*math.Cos(Y[0])/math.Sin(Y[0]+Y[4])) + Arm_I + CW_I + Arm_Mass*math.Pow(L_Arm_CG,2) +Proj_Mass*math.Pow(L_Arm_Lo,2)*math.Pow(math.Sin(Y[0]),2)/math.Pow(math.Sin(Y[0]+Y[4]),2) + CW_Mass*(math.Pow(L_Arm_Sh,2)+math.Pow(L_Arm_We,2)+2*L_Arm_Sh*L_Arm_We*math.Cos(Y[2]))
  
  M12 := CW_I + L_Arm_We*CW_Mass*(L_Arm_We+L_Arm_Sh*math.Cos(Y[2]))
  
  M21 := M12
  
  M22 := CW_I + CW_Mass*math.Pow(L_Arm_We,2)
  
  r1 := g*L_Arm_CG*Arm_Mass*math.Sin(Y[0]) +
		L_Arm_Lo*L_Sling*Proj_Mass*(math.Sin(Y[4])*math.Pow((Y[1]+Y[5]),2)+math.Cos(Y[4])*
		(math.Cos(Y[0]+Y[4])*Y[5]*(Y[5]+2*Y[1])/math.Sin(Y[0]+Y[4])+
		(math.Cos(Y[0]+Y[4])/math.Sin(Y[0]+Y[4])+L_Arm_Lo*math.Cos(Y[0])/(L_Sling*math.Sin(Y[0]+Y[4])))*math.Pow(Y[1],2))) +
		L_Arm_Lo*Proj_Mass*math.Sin(Y[0])*(L_Arm_Lo*math.Sin(Y[4])*math.Pow(Y[1],2)-L_Sling*(math.Cos(Y[0]+Y[4])*Y[5]*(Y[5]+2*Y[1])/math.Sin(Y[0]+Y[4])+
		(math.Cos(Y[0]+Y[4])/math.Sin(Y[0]+Y[4])+L_Arm_Lo*math.Cos(Y[0])/(L_Sling*math.Sin(Y[0]+Y[4])))*math.Pow(Y[1],2)))/math.Sin(Y[0]+Y[4]) -
		g*CW_Mass*(L_Arm_Sh*math.Sin(Y[0])+L_Arm_We*math.Sin(Y[0]+Y[2])) - L_Arm_Sh*L_Arm_We*CW_Mass*math.Sin(Y[2])*(math.Pow(Y[1],2)-math.Pow((Y[1]+Y[3]),2))
  
  r2 := (-1.0)*L_Arm_We*CW_Mass*(g*math.Sin(Y[0]+Y[2])+L_Arm_Sh*math.Sin(Y[2])*math.Pow(Y[1],2))

  //Return fuction returns Aq', Aq'',Wq', Wq'', Sq',Sq''

  ret[0] = Y[1]
  ret[1] = (r1*M22-r2*M12)/(M11*M22-M12*M21)
  ret[2] = Y[3]
  ret[3] = (-1.0)*(r1*M21-r2*M11)/(M11*M22-M12*M21)
  ret[4] = Y[5]
  ret[5] = (-1.0)*math.Cos(Y[0]+Y[4])*Y[5]*(Y[5]+2*Y[1])/math.Sin(Y[0]+Y[4]) - (math.Cos(Y[0]+Y[4])/math.Sin(Y[0]+Y[4])+L_Arm_Lo*math.Cos(Y[0])/(L_Sling*math.Sin(Y[0]+Y[4])))*math.Pow(Y[1],2) - (L_Arm_Lo*math.Sin(Y[0])+L_Sling*math.Sin(Y[0]+Y[4]))*ret[1]/(L_Sling*math.Sin(Y[0]+Y[4]))
  

  return ret

}

// Simulation 1 Stop function
// Force in +Y dir at projectile point
func Fy(Y []float64) bool {
	// Y = [
			// aq
			// aq'
			// wq
			// wq'
			// sq
			// sq'
			// ]
	deriv := treb_1(1.0,Y) //Return fuction returns aq', aq'',wq', wq'', sq',sq''
	Fy := Proj_Mass*(g+(L_Sling*(math.Cos(Y[0]+Y[4])*deriv[4]*(deriv[4]+2*deriv[0])/math.Sin(Y[0]+Y[4])+(math.Cos(Y[0]+Y[4])/math.Sin(Y[0]+Y[4])+L_Arm_Lo*math.Cos(Y[0])/(L_Sling*math.Sin(Y[0]+Y[4])))*math.Pow(deriv[0],2))-L_Arm_Lo*math.Sin(Y[4])*math.Pow(deriv[0],2)-L_Arm_Lo*(math.Cos(Y[4])-math.Sin(Y[0])/math.Sin(Y[0]+Y[4]))*deriv[1])/math.Sin(Y[0]+Y[4]))
	
	if Fy < 0 {
		return true
	}else{
		return false
	}
}



// Second Simulation
func treb_2(time float64, Y []float64) []float64 {
	ret := make([]float64,len(Y))
	
	M11 := Arm_I + CW_I + Arm_Mass * math.Pow(L_Arm_CG,2) + Proj_Mass*(math.Pow(L_Arm_Lo,2)+math.Pow(L_Sling,2)+2*L_Arm_Lo*L_Sling*math.Cos(Y[5])) + CW_Mass*(math.Pow(L_Arm_Sh,2)+math.Pow(L_Arm_We,2)+2*L_Arm_Sh*L_Arm_We*math.Cos(Y[2]))
	M12 := CW_I + L_Arm_We * CW_Mass * (L_Arm_We + L_Arm_Sh * math.Cos(Y[2]))
	M13 := L_Sling * Proj_Mass * (L_Sling + L_Arm_Lo * math.Cos(Y[4]))
	M21 := CW_I * L_Arm_We * CW_Mass * (L_Arm_We + L_Arm_Sh * math.Cos(Y[2]))
	M22 := CW_I + CW_Mass * math.Pow(L_Arm_We,2)
	M31 := L_Sling * Proj_Mass * (L_Sling + L_Arm_Lo * math.Cos(Y[4]))
	M33 := Proj_Mass * math.Pow(L_Sling,2)
	
	r1 := g*L_Arm_CG*Arm_Mass*math.Sin(Y[0]) + g*Proj_Mass*(L_Arm_Lo*math.Sin(Y[0])+L_Sling*math.Sin(Y[0]+Y[4])) - g*CW_Mass*(L_Arm_Sh*math.Sin(Y[0])+L_Arm_We*math.Sin(Y[0]+Y[2])) - L_Arm_Lo*L_Sling*Proj_Mass*math.Sin(Y[4])*(math.Pow(Y[1],2)-math.Pow((Y[1]+Y[5]),2)) - L_Arm_Sh*L_Arm_We*CW_Mass*math.Sin(Y[2])*(math.Pow(Y[1],2)-math.Pow((Y[1]+Y[3]),2))
	
	r2 := -L_Arm_We*CW_Mass*(g*math.Sin(Y[0]+Y[2])+L_Arm_Sh*math.Sin(Y[2])*math.Pow(Y[1],2))
	
	r3 := L_Sling*Proj_Mass*(g*math.Sin(Y[0]+Y[4])-L_Arm_Lo*math.Sin(Y[4])*math.Pow(Y[1],2))
	
	//Return fuction returns Aq', Aq'',Wq', Wq'', Sq',Sq''
	
	ret[0] = Y[1]
	ret[1] = -(r1*M22*M33-r2*M12*M33-r3*M13*M22)/(M13*M22*M31-M33*(M11*M22-M12*M21))
	ret[2] = Y[3]
	ret[3] = (r1*M21*M33-r2*(M11*M33-M13*M31)-r3*M13*M21)/(M13*M22*M31-M33*(M11*M22-M12*M21))
	ret[4] = Y[5]
	ret[5] = (r1*M22*M31-r2*M12*M31-r3*(M11*M22-M12*M21))/(M13*M22*M31-M33*(M11*M22-M12*M21))
	
	return ret
	
}

// Second simulation stop function
var cnt int = 0
// Velocity vector > launch angle
func Launch(Y []float64) bool {
	xV := -L_Sling * math.Cos(Y[0] + Y[4]) * (Y[1] + Y[5]) - L_Arm_Lo * math.Cos(Y[0]) * Y[1]
	yV := -L_Sling * math.Sin(Y[0] + Y[4]) * (Y[1] + Y[5]) - L_Arm_Lo * math.Sin(Y[0]) * Y[1]

	cur_angle := math.Atan(yV/xV)
	if Rel_Angle - 0.1 <= cur_angle && cur_angle <= Rel_Angle + 0.1 && yV > 0.0 && xV > 0.0{
		return true
	}else{
		return false
	}
}

// Third Simulation
func treb_3(time float64, Y []float64) []float64 {

	ret := make([]float64,len(Y))

	ret[0] = Y[1]
	ret[1] = -(ro * Cd * Area * (Y[1]-Wind_v) * math.Sqrt(math.Pow(Y[3],2) + math.Pow((Wind_v - Y[1]),2)))/(2*Proj_Mass)
	ret[2] = Y[3]
	ret[3] = -g - (ro * Cd * Area * Y[3] * math.Sqrt(math.Pow(Y[3],2) + math.Pow((Wind_v - Y[1]),2)))/(2*Proj_Mass)

	return ret
}

// Third simulation stop function
func Proj_Y(Y []float64) bool {
	if Y[2] <= 0 {
		return true
	}else {
		return false
	}
}


/////////////////////////////
// Projectile angles to xy //
/////////////////////////////

func ProjXY(Y [][]float64) [][]float64 {
	XY := [][]float64{}

	for i:=0; i<len(Y); i++ {
		temp := make([]float64,3)
		//Put in the Time
		temp[0] = Y[i][0]
		//X value
		temp[1] = - L_Arm_Lo * math.Sin(Y[i][1]) - L_Sling * math.Sin(Y[i][1] + Y[i][5])
		//Y value
		temp[2] = L_Arm_Lo * math.Cos(Y[i][1]) + L_Sling * math.Cos(Y[i][1] + Y[i][5])
		// Append
		XY = append(XY,temp)
	}

	return XY
}

// Y = [
// Aq
// Aq'
// Wq
// Wq'
// Sq
// Sq'
// ]

//////////////////////////////
// Projectile Velocity in XY//
//////////////////////////////

func ProjV(Y []float64) []float64 {
	xP := - L_Arm_Lo * math.Sin(Y[0]) - L_Sling * math.Sin(Y[0] + Y[4])
	xV := -L_Sling * math.Cos(Y[0] + Y[4]) * (Y[1] + Y[5]) - L_Arm_Lo * math.Cos(Y[0]) * Y[1]
	yP := L_Arm_Lo * math.Cos(Y[0]) + L_Sling * math.Cos(Y[0] + Y[4])
	yV := -L_Sling * math.Sin(Y[0] + Y[4]) * (Y[1] + Y[5]) - L_Arm_Lo * math.Sin(Y[0]) * Y[1]
	return []float64{xP,xV,yP,yV}
}
/////////////////////
// CW angles to xy //
/////////////////////

func CWXY(Y [][]float64) [][]float64 {
	XY := [][]float64{}

	for i:=0; i<len(Y); i++ {
		temp := make([]float64,3)
		//Put in the Time
		temp[0] = Y[i][0]
		//X value
		temp[1] = L_Arm_Sh * math.Sin(Y[i][1]) + L_Arm_We * math.Sin(Y[i][1] + Y[i][3])
		//Y value
		temp[2] = -L_Arm_Sh * math.Cos(Y[i][1]) - L_Arm_We * math.Cos(Y[i][1] + Y[i][3])
		// Append
		XY = append(XY,temp)
	}

	return XY
}

////////////////////////////
// Arm Sling angles to XY //
////////////////////////////

func Arm_Sl_XY(Y [][]float64) [][]float64 {
	XY := [][]float64{}

	for i:=0; i<len(Y); i++ {
		temp := make([]float64,3)
		//Put in the Time
		temp[0] = Y[i][0]
		//X value
		temp[1] = -L_Arm_Lo * math.Sin(Y[i][1])
		//Y value
		temp[2] = L_Arm_Lo * math.Cos(Y[i][1])
		// Append
		XY = append(XY,temp)
	}

	return XY
}

//////////////////////
// Weight Arm Point //
//////////////////////

func Weight_Arm_XY(Y [][]float64) [][]float64 {
	XY := [][]float64{}

	for i:=0; i<len(Y); i++ {
		temp := make([]float64,3)
		//Put in the Time
		temp[0] = Y[i][0]
		//X value
		temp[1] = L_Arm_Sh * math.Sin(Y[i][1])
		//Y value
		temp[2] = -L_Arm_Sh * math.Cos(Y[i][1])
		// Append
		XY = append(XY,temp)
	}

	return XY

}



//////////////////////////
// Runge Kutta Fehlberg //
//////////////////////////

func rk45(from, h, to, max_h float64, y []float64, fn func(float64, []float64) []float64, stop_fn func([]float64) bool) [][]float64 {
  eps := 0.000001
	var parameters = len(y)

	// initialize 'outer slice'
	ySlice := [][]float64{}
	// initialize first 'inner slice'
	ySlice = append(ySlice,make([]float64, parameters+1))

	// fill with start values
	ySlice[0][0] = from
	for i := 0; i < parameters; i++ {
		ySlice[0][i+1] = y[i]
	}

	var k1 = make([]float64, parameters)
	var k2 = make([]float64, parameters)
	var k3 = make([]float64, parameters)
	var k4 = make([]float64, parameters)
	var k5 = make([]float64, parameters)
	var k6 = make([]float64, parameters)

	var yn = make([]float64, parameters)

	//
	var k2p = make([]float64, parameters)
	var k3p = make([]float64, parameters)
	var k4p = make([]float64, parameters)
	var k5p = make([]float64, parameters)
	var k6p = make([]float64, parameters)
	
  t := from;
	for  t < to {
	  step := len(ySlice)

		// initialize
		for value := 0; value < parameters; value++ {
			yn[value] = ySlice[step-1][value+1]
		}
    
    	t1 := t
    
		// generate k1
		for value := 0; value < parameters; value++ {
			k1[value] = h * fn(t1, yn)[value]
		}
		

		// generate the parameter for k2
		for value := 0; value < parameters; value++ {
			k2p[value] = yn[value] + 0.25*k1[value]
		}
		
		t2 := t + 0.25 * h

		// generate k2
		for value := 0; value < parameters; value++ {
			k2[value] = h * fn(t2, k2p)[value]
		}

		// generate the parameter for k3
		for value := 0; value < parameters; value++ {
			k3p[value] = yn[value] + (3.0/32.0) * k1[value] + (9.0/32.0) * k2[value]
		}
		
		t3 := t + (3.0/8.0) * h

		// generate k3
		for value := 0; value < parameters; value++ {
			k3[value] = h * fn(t3, k3p)[value]
		}

		// generate the parameter for k4
		for value := 0; value < parameters; value++ {
			k4p[value] = yn[value] + (1932.0/2197.0) * k1[value] - (7200.0/2197) * k2[value] + (7296.0/2197.0) * k3[value]
		}

		t4 := t + (12.0/13.0) * h

		// generate k4
		for value := 0; value < parameters; value++ {
			k4[value] = h * fn(t4, k4p)[value]
		}
		
		// generate the parameter for k5
		for value := 0; value < parameters; value++ {
		  k5p[value] = yn[value] + (439.0/216.0) * k1[value] - 8.0 * k2[value] + (3680.0/513.0) * k3[value] - (845.0/4104.0) * k4[value]
		}

    t5 := t + h
    
    // generate k5
    for value := 0; value < parameters; value ++ {
      k5[value] = h * fn(t5,k5p)[value]
    }
    
    // generate the parameter for k6
    for value := 0; value < parameters; value ++ {
      k6p[value] = yn[value] - (8.0/27.0) * k1[value] + 2.0 * k2[value] - (3544.0/2565.0) * k3[value] + (1859.0/4104.0) * k4[value] - (11.0/40.0) * k5[value]
    }
    
    t6 := t + 0.5 * h
    
    // generate k6
    for value := 0; value < parameters; value ++ {
      k6[value] = h * fn(t6,k6p)[value]
    }
    
		
    // generate yk
    var yk = make([]float64, parameters)
    
    // generate the values for yk
    for value := 0; value < parameters; value ++ {
      yk[value] = yn[value] + (25.0/216.0) * k1[value] + (1408.0/2565.0) * k3[value] + (2197.0/4104.0) * k4[value] - (1.0/5.0) * k5[value]
    }
    
    
    //generate zk
    var zk = make([]float64, parameters)
    
    // generate the values for zk
    for value := 0; value < parameters; value ++ {
      zk[value] = yn[value] + (16.0/135.0) * k1[value] + (6656.0/12825.0) * k3[value] + (28561.0/56430.0) * k4[value] - (9.0/50.0) * k5[value] + (2.0/55.0) * k6[value]
    }
    
    
    // generate the residual
    L2_R := 0.0
    
    for value := 0; value < parameters; value ++ {
      L2_R += math.Pow(zk[value] - yk[value],2)
    }
    
    
    L2_R = math.Sqrt(L2_R)
    
    if L2_R <= eps {
      
      t = t + h
      
      // initialize 'inner slice'
		  ySlice = append(ySlice,make([]float64, parameters+1))
      
      ySlice[step][0] = t
      
      // Error small enough. Save yk slice to global
      for value := 0; value < parameters; value ++ {
        ySlice[step][value+1] = yk[value]
      }
      
    }
    
    // Find the scalar multiplier for the time step
    d := 0.84 * math.Pow(((eps*h)/L2_R),0.25)
    
    // Calculate the timestep
    h = d*h
    
    // If the timestep will put you over the final time, choose the difference as the timestep. 
    // Or if the timestep is larger than the largest specified timestep change to the largest timestep.
    if t + h > to {
      h = to-t
    }else if h > max_h {
    	h = max_h
    }
    
    if stop_fn(yk) == true {
    	break
    }
    
    
	}
	return ySlice
}



func checkError(message string, err error) {
    if err != nil {
        log.Fatal(message, err)
    }
}

func Max(a []float64) float64{
  max := a[0]
  for i := 0; i < len(a); i++ {
    if a[i] > max {
      max = a[i]
    }
  }
  
  return max
}

// Y = [
// Aq
// Aq'
// Wq
// Wq'
// Sq
// Sq'
// ]

func main() {
	start := time.Now()
	IV := []float64{2.3562,0,-2.3562,0,2.3562,0}
  	y1 := rk45(0, 0.0001,5,0.5,IV,treb_1,Fy)
	y2 := rk45(0, 0.0001,1,0.5, y1[len(y1)-1][1:], treb_2,Launch)
	IV3 := ProjV(y2[len(y2)-1][1:])
	y3 := rk45(0, 0.001,15.0,50.0,IV3,treb_3,Proj_Y)
	fmt.Println("Range: ",y3[len(y3)-1][1])

	file, err := os.Create("result.csv")
    checkError("Cannot create file", err)
    defer file.Close()

    writer := csv.NewWriter(file)
    xy_proj := ProjXY(y2)
    xy_cw := CWXY(y2)
    xy_arm_sling := Arm_Sl_XY(y2)
    xy_weight_arm := Weight_Arm_XY(y2)
    pos := [][]string{}
    for i:=0; i<len(y2); i++{
        time:=strconv.FormatFloat(xy_proj[i][0], 'f', 8, 64)
        xp := strconv.FormatFloat(xy_proj[i][1],'f',8,64)
        yp := strconv.FormatFloat(xy_proj[i][2],'f',8,64)
        xcw := strconv.FormatFloat(xy_cw[i][1],'f',8,64)
        ycw := strconv.FormatFloat(xy_cw[i][2],'f',8,64)
        xasl := strconv.FormatFloat(xy_arm_sling[i][1],'f',8,64)
        yasl := strconv.FormatFloat(xy_arm_sling[i][2],'f',8,64)
        xwa := strconv.FormatFloat(xy_weight_arm[i][1],'f',8,64)
        ywa := strconv.FormatFloat(xy_weight_arm[i][2],'f',8,64)
        temp := []string{time,xp,yp,xcw,ycw,xasl,yasl,xwa,ywa}
        pos = append(pos,temp)
    }

    for _, value := range pos {
        err := writer.Write(value)
        checkError("Cannot write to file", err)
    }

    defer writer.Flush()

	elapsed := time.Since(start)
	fmt.Println(elapsed)

}


