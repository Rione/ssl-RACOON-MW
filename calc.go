package main

import "math"

func Calc_degree_normalize(rad float32) float32 {
	if rad > math.Pi {
		rad -= 2 * math.Pi
	} else if rad < -math.Pi {
		rad += 2 * math.Pi
	}
	return rad
}

func Calc_degree(x_1 float32, y_1 float32, x_2 float32, y_2 float32) float32 {
	var diff_x float64 = float64(x_1 - x_2)
	var diff_y float64 = float64(y_1 - y_2)
	var rad float32 = float32(math.Atan2(diff_y, diff_x))
	return rad
}

func Calc_distance(x_1 float32, y_1 float32, x_2 float32, y_2 float32) float32 {

	var diff_x float64 = float64(x_1 - x_2)
	var diff_y float64 = float64(y_1 - y_2)

	var dist float32 = float32(math.Sqrt((diff_x * diff_x) + (diff_y * diff_y)))

	return dist
}
