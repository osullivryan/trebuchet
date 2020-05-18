package utils

import "log"

func CheckError(message string, err error) {
	if err != nil {
		log.Fatal(message, err)
	}
}

func Max(a []float64) float64 {
	max := a[0]
	for i := 0; i < len(a); i++ {
		if a[i] > max {
			max = a[i]
		}
	}

	return max
}
