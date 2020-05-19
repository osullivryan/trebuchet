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

var trebuchetConfiguration = types.TrebuchetParameters{
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

var projectileConfiguration = types.ProjectileParameters{
	Proj_mass: 0.15,
	Proj_dia:  0.076,
	Cd:        0.4,
	Area:      math.Pi / 4.0 * math.Pow(0.076, 2),
}

var globalConfiguration = types.GlobalParameters{
	G:      9.81,
	Ro:     1.225,
	Wind_v: 0.0,
}

func main() {
	start := time.Now()

	simulationEnvironment := types.SimulationEnvironment{
		Trebuchet:  trebuchetConfiguration,
		Global:     globalConfiguration,
		Projectile: projectileConfiguration,
	}

	firstPhase := ode_solver.Simulation{
		Functions: types.SimulationFunction{
			Evaluation_function:       trebuchet_setup.Phase_1,
			Integration_stop_function: trebuchet_setup.Fy,
		},
		Environments: simulationEnvironment,
	}

	secondPhase := ode_solver.Simulation{
		Functions: types.SimulationFunction{
			Evaluation_function:       trebuchet_setup.Phase_2,
			Integration_stop_function: trebuchet_setup.Launch,
		},
		Environments: simulationEnvironment,
	}

	thirdPhase := ode_solver.Simulation{
		Functions: types.SimulationFunction{
			Evaluation_function:       trebuchet_setup.Phase_3,
			Integration_stop_function: trebuchet_setup.Proj_Y,
		},
		Environments: simulationEnvironment,
	}

	// The initial values
	// Solve the first phase
	IV1 := []float64{2.3562, 0, -2.3562, 0, 2.3562, 0}
	y1 := firstPhase.Solve(0, 0.0001, 5.0, 0.5, IV1)

	// Solve the second phase
	IV2 := y1[len(y1)-1][1:]
	y2 := secondPhase.Solve(0, 0.0001, 1, 0.5, IV2)

	IV3 := trebuchet_setup.ProjV(y2[len(y2)-1][1:], simulationEnvironment)
	fmt.Println("IV3 ", IV3)
	y3 := thirdPhase.Solve(0, 0.0001, 15, 0.005, IV3)

	fmt.Println("Range: ", y3[len(y3)-1][1])

	utils.WriteResults("results.csv", simulationEnvironment, y2)

	elapsed := time.Since(start)
	fmt.Println(elapsed)

}
