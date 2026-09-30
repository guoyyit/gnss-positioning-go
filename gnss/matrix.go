package gnss

// invert4 returns the inverse of a 4x4 matrix via Gauss-Jordan elimination.
func invert4(a [4][4]float64) ([4][4]float64, bool) {
	var inv [4][4]float64
	for i := 0; i < 4; i++ {
		inv[i][i] = 1
	}
	m := a
	for col := 0; col < 4; col++ {
		pivot := col
		best := abs(m[col][col])
		for r := col + 1; r < 4; r++ {
			if abs(m[r][col]) > best {
				best = abs(m[r][col])
				pivot = r
			}
		}
		if best < 1e-14 {
			return inv, false
		}
		if pivot != col {
			m[col], m[pivot] = m[pivot], m[col]
			inv[col], inv[pivot] = inv[pivot], inv[col]
		}
		p := m[col][col]
		for j := 0; j < 4; j++ {
			m[col][j] /= p
			inv[col][j] /= p
		}
		for r := 0; r < 4; r++ {
			if r == col {
				continue
			}
			f := m[r][col]
			for j := 0; j < 4; j++ {
				m[r][j] -= f * m[col][j]
				inv[r][j] -= f * inv[col][j]
			}
		}
	}
	return inv, true
}

// solveSym4 solves A x = b for a symmetric 4x4 matrix.
func solveSym4(a [4][4]float64, b [4]float64) ([4]float64, bool) {
	inv, ok := invert4(a)
	if !ok {
		return [4]float64{}, false
	}
	var x [4]float64
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			x[i] += inv[i][j] * b[j]
		}
	}
	return x, true
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
