package utils

import (
	"encoding/csv"
	"os"
	"strconv"

	"github.com/osullivryan/trebuchet/trebuchet_setup"
	"github.com/osullivryan/trebuchet/types"
)

func WriteResults(file_name string, simulation_environment types.SimulationEnvironment, result_arrays ...[][]float64) {

	file, err := os.Create(file_name)
	utils.CheckError("Cannot create file", err)
	defer file.Close()

	writer := csv.NewWriter(file)
	pos := [][]string{}

	for _, sim_result := range result_arrays {
		xy_proj := trebuchet_setup.ProjXY(sim_result, simulation_environment)
		xy_cw := trebuchet_setup.CWXY(sim_result, simulation_environment)
		xy_arm_sling := trebuchet_setup.Arm_Sl_XY(sim_result, simulation_environment)
		xy_weight_arm := trebuchet_setup.Weight_Arm_XY(sim_result, simulation_environment)
		for i := 0; i < len(sim_result); i++ {
			time := strconv.FormatFloat(xy_proj[i][0], 'f', 8, 64)
			xp := strconv.FormatFloat(xy_proj[i][1], 'f', 8, 64)
			yp := strconv.FormatFloat(xy_proj[i][2], 'f', 8, 64)
			xcw := strconv.FormatFloat(xy_cw[i][1], 'f', 8, 64)
			ycw := strconv.FormatFloat(xy_cw[i][2], 'f', 8, 64)
			xasl := strconv.FormatFloat(xy_arm_sling[i][1], 'f', 8, 64)
			yasl := strconv.FormatFloat(xy_arm_sling[i][2], 'f', 8, 64)
			xwa := strconv.FormatFloat(xy_weight_arm[i][1], 'f', 8, 64)
			ywa := strconv.FormatFloat(xy_weight_arm[i][2], 'f', 8, 64)
			temp := []string{time, xp, yp, xcw, ycw, xasl, yasl, xwa, ywa}
			pos = append(pos, temp)
		}
	}

	for _, value := range pos {
		err := writer.Write(value)
		utils.CheckError("Cannot write to file", err)
	}

	defer writer.Flush()

}
