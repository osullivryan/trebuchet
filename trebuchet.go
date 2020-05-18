package main

import (
	"fmt"
	"math"
	"time"

	"github.com/osullivryan/trebuchet/ode_solver"
	"github.com/osullivryan/trebuchet/trebuchet_setup"
	"github.com/osullivryan/trebuchet/types"
	"github.com/osullivryan/trebuchet/utils"
)

var trebuchet_configuration = types.TrebuchetParameters{
	L_arm_lo:  6.792,
	L_arm_sh:  1.75,
	L_arm_we:  2.0,
	L_sling:   6.833,
	H_pivot:   5.0,
	Cw_mass:   550.0,
	Cw_i:      1.0,
	Arm_mass:  10.0,
	Arm_i:     65.0,
	L_arm_cg:  2.52,
	Rel_angle: .785398,
}

var projectile_configuration = types.ProjectileParameters{
	Proj_mass: 0.15,
	Proj_dia:  0.076,
	Cd:        0.4,
	Area:      math.Pi * math.Pow(0.5, 2.0),
}

var global_configuration = types.GlobalParameters{
	G:      9.81,
	Ro:     1.225,
	Wind_v: 0.0,
}

func main() {
	start := time.Now()

	simulation_environment := types.SimulationEnvironment{
		Trebuchet:  trebuchet_configuration,
		Global:     global_configuration,
		Projectile: projectile_configuration,
	}

	first_phase := ode_solver.Simulation{
		Functions: types.SimulationFunction{
			Evaluation_function:       trebuchet_setup.Phase_1,
			Integration_stop_function: trebuchet_setup.Fy,
		},
		Environments: simulation_environment,
	}

	second_phase := ode_solver.Simulation{
		Functions: types.SimulationFunction{
			Evaluation_function:       trebuchet_setup.Phase_2,
			Integration_stop_function: trebuchet_setup.Launch,
		},
		Environments: simulation_environment,
	}

	third_phase := ode_solver.Simulation{
		Functions: types.SimulationFunction{
			Evaluation_function:       trebuchet_setup.Phase_3,
			Integration_stop_function: trebuchet_setup.Proj_Y,
		},
		Environments: simulation_environment,
	}

	// The initial values
	// Solve the first phase
	IV1 := []float64{2.3562, 0, -2.3562, 0, 2.3562, 0}
	y1 := first_phase.Solve(0, 0.0001, 5.0, 0.5, IV1)

	// Solve the second phase
	IV2 := y1[len(y1)-1][1:]
	y2 := second_phase.Solve(0, 0.0001, 1, 0.5, IV2)

	IV3 := trebuchet_setup.ProjV(y2[len(y2)-1][1:], simulation_environment)
	y3 := third_phase.Solve(0, 0.0001, 1, 0.5, IV3)

	fmt.Println("Range: ", y3[len(y3)-1][1])

	utils.WriteResults("results.csv", simulation_environment, y1, y2, y3)

	elapsed := time.Since(start)
	fmt.Println(elapsed)

}
