package types

type TrebuchetParameters struct {

	// Length of long arm from pivot to sling attachment point
	L_arm_lo float64

	// Length of short arm from pivot to weight attachment point
	L_arm_sh float64

	// Lenght of weight arm
	L_arm_we float64

	// Lenght of sling
	L_sling float64

	// Height of pivot
	H_pivot float64

	// Mass of Counter Weight
	Cw_mass float64

	// Inertia of CW_Mass
	Cw_i float64

	// Mass of Long Arm
	Arm_mass float64

	//Inertial of Long arm
	Arm_i float64

	// Lenght from pivot to CG of long arm
	L_arm_cg float64

	// Release angle (radian)
	Rel_angle float64
}

type ProjectileParameters struct {
	// Projectile Mass
	Proj_mass float64

	// Projectile Diameter
	Proj_dia float64

	// Drag Coeff
	Cd float64

	Area float64
}

type GlobalParameters struct {
	G      float64
	Wind_v float64
	Ro     float64
}

type SimulationEnvironment struct {
	Trebuchet  TrebuchetParameters
	Projectile ProjectileParameters
	Global     GlobalParameters
}

type SimulationFunction struct {
	Evaluation_function       func(float64, []float64, SimulationEnvironment) []float64
	Integration_stop_function func([]float64, SimulationEnvironment) bool
}
