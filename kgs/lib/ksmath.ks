@lazyglobal off.

//
// math function library
//

// returns solution to linear equation Ax=b solved by the gauss jordan algorithm
function gauss_jordan {
    
    parameter abi.

    local ab is deep_copy(abi).
    local n is ab:length.
	
    // guassian elimination with partial pivoting
    for row in range(n - 1) {
        // select the row with the largest pivot
		local pivot_row is row.
		local pivot_value is ab[row][row].
        local abs_pivot_value is abs(pivot_value).
        for i in range(row + 1, n) {
			if abs(ab[i][row]) > abs_pivot_value {
				set pivot_row to i.
				set pivot_value to ab[i][row].
			}
		}
		
        // exchange rows
		if pivot_row <> row {			
			local temp is ab[pivot_row].
			set ab[pivot_row] to ab[row].
			set ab[row] to temp.
		}
		
        // eliminate values below the pivot, only values not in the pivot
        // column are actually changed
        for i in range(row + 1, n) {
			local m is ab[i][row] / pivot_value.	
            for j in range(row + 1, n + 1) {
				set ab[i][j] to ab[i][j] - m * ab[row][j].
			}
		}
	}

    // elimate values above the pivots to reach reduced row echelon form, only
    // the values in the last column are actually changed
    for row in range(n - 1, -1)	{
		set ab[row][n] to ab[row][n] / ab[row][row].
        for i in range(row - 1, -1) {
			set ab[i][n] to ab[i][n] - ab[i][row] * ab[row][n].
		}
	}

    // the solution is now in the last column of the matrix
	local x is list().
    for i in range(n) {
		x:add(ab[i][n]).
	}

	return x.
}

// returns solution to linear equation Ax=b solved by a gauss jordan algorithm, if
// system is overdetermined returns minimum norm solution
function matrix_solve {

	parameter n.
	parameter m.
	parameter ab.

	// if n > m calculate minimum norm (w = (AA^T)^-1 b, x = A^T w) otherwise solve
	// directly (x = A^-1 b)
	if n > m {
		// calculate AA^T with b appended
		local aatb is list().
		for i in range(m) {	
			local row is list().
			for j in range(m) {
				row:add(0).
				for k in range(n) {
					set row[j] to row[j] + ab[i][k] * ab[j][k].
				}
			}
			row:add(ab[i][n]).
			aatb:add(row).
		}

		// solve for w
		local w is gauss_jordan(aatb).

		// solve for x
		local x is list().
		for i in range(n) {
			local val is 0.
			for j in range(m) {
				set val to val + ab[j][i] * w[j].
			}
			x:add(val).
		}
		
		return x.
	}
	else {
		// solve for x
		return gauss_jordan(ab).
	}
}

// solves for the root of a multivariate non linear problem using newton's method
function newton {

	parameter fdf_function.
	parameter xi.
	parameter p.
	parameter eps.
	parameter debug_function is 0.

	// calculate first jacobian matrix and error vector, then solve for dx
	local x is xi:copy.
    local x_new is xi:copy.
    local fdf is fdf_function(x, p).
	local n is xi:length.
	local m is fdf:length.
	local dx is matrix_solve(n, m, fdf).

	// calculate max error
    local fmax is 0.
    local fmax_new is 0.
    for i in range(m) {
        set fmax to max(fmax, abs(fdf[i][n])).
    }

	// limit beta if error is significant
	local beta is min(1, 0.2 / fmax).

	// iteration loop
	local iteration is 0.

	until false {
		set iteration to iteration + 1.

		// if there is a debug function call it
		if debug_function:typename = "userdelegate" {
			debug_function(x, p, iteration, x_new, fdf, fmax).
		}
		
		// calculate new x
		for i in range(n) {
			set x_new[i] to x[i] - beta * dx[i].
		}

		// if error is below tolerance return new x or if max iterations reached return x
        if fmax < eps {
			return list(x_new, iteration, fmax).
		}
		else if iteration > 19 {
			return list(x, iteration, fmax).
		}

		// calculate new jacobian matrix and error vector
        set fdf to fdf_function(x_new, p).

		// calculate new max error
        set fmax_new to 0.0.
		for i in range(m) {
			set fmax_new to max(fmax_new, abs(fdf[i][n])).
		}
        
		// if error has been reduced accept new x and solve for a new dx otherwise reduce beta
		if fmax_new > fmax {
            local r is fmax_new / fmax.
			set beta to beta * sqrt(1 + 6 * r) / (3 * r).
		}
		else {
			set x to x_new:copy.
            set dx to matrix_solve(n, m, fdf).
			set fmax to fmax_new.
			set beta to min(1, 0.2 / fmax).
		}
	}
}

// calculate jacobian numerically using finite difference
function numerical_jacobian {

	parameter f_function.
	parameter x.
	parameter p.

	// calculate error vector for given value of x
	local fdf is list().
	local f_output is f_function(x, p).

	local b is list().
	for i in range(f_output:length) {
		b:add(f_output[i][f_output[i]:length - 1]).
	}

	// calculate error vector for perterbed value of x and estimate derivative using
	// finite difference then populate jacobian
	for i in range(f_output:length) {
		fdf:add(list()).
	}

	for i in range(x:length) {
		local xp is x:copy.
		set xp[i] to xp[i] + 1e-4.
		local fp_output is f_function(xp, p).
		for j in range(fp_output:length) {
			fdf[j]:add((fp_output[j][f_output[j]:length - 1] - b[j]) / 1e-4).
		}
	}

	// append error vector to jacobian
	for i in range(f_output:length) {
		fdf[i]:add(f_output[i][f_output[i]:length - 1]).
	}

	return fdf.
}

// returns solution to initial value problem with n equations using runge kutta method (4th order with 5th order error control)
function rkf45 {

    parameter df_function.
	parameter p.
	parameter n.
	parameter t.
	parameter tf.
	parameter yi.
	parameter settings.
	
	local y is deep_copy(yi).
	local steps is max(floor((tf - t) / settings:max_step), 0) + 1.
	local h is (tf - t) / steps.
	local eps is settings:eps_rkf45.
	local l is 6.
	local last is false.

	local c1 is list(0, 1 / 4, 3 / 8, 12 / 13, 1, 1 / 2).
	local c2 is list(
		list(0, 0, 0, 0, 0),
		list(1 / 4, 0, 0, 0, 0),
		list(3 / 32, 9 / 32, 0, 0, 0),
		list(1932 / 2197, -7200 / 2197, 7296 / 2197, 0, 0),
		list(439 / 216, -8, 3680 / 513, -845 / 4104, 0),
		list(-8 / 27, 2, -3544 / 2565, 1859 / 4104, -11 / 40)
	).
	local c3 is list(25 / 216, 0, 1408 / 2565, 2197 / 4104, -1 / 5, 0).
	local c4 is list(16 / 135, 0, 6656 / 12825, 28561 / 56430, -9 / 50, 2 / 55).

	until false {

		local df is list().
        for i in range(l) {

			local tm is t + c1[i] * h.
			local ym is deep_copy(y).
			
			for j in range(0, i) {
				for k in range(n) {
					set ym[k] to ym[k] + h * c2[i][j] * df[j][k].
				}
			}

			df:add(df_function(tm, ym, p)).
		}
		
		local yf is y:copy.
		local zf is y:copy.
		
		for i in range(l) {
			for j in range(n) {
				set yf[j] to yf[j] + h * c3[i] * df[i][j].
				set zf[j] to zf[j] + h * c4[i] * df[i][j].
			}
		}
		
		local max_err is 0.
		
		for i in range(n) {
			local err is zf[i] - yf[i].
			if yf[i]:typename = "vector" {
				set err to err:mag.
			}
			else {
				set err to abs(err).
			}
			if err > max_err {
				set max_err to err.
			}
		}
		
		if max_err < eps {
			set t to t + h.
			set y to zf.
			
			if last {
				return list(t, y, h).
			}
		}
		
		set h to h * 0.84 * (eps / (max_err + 1e-12)) ^ 0.25.
		
		if t + h > tf {
			set h to tf - t.
			set last to true.
		}
	}
}

// returns solution to initial value problem with n equations using runge kutta method (4th order)
function rk4 {

    parameter df_function.
	parameter p.
	parameter n.
	parameter t.
	parameter tf.
	parameter yi.
	parameter settings.
	
	local y is deep_copy(yi).
	local steps is max(floor((tf - t) / settings:max_step), 0) + 1.
	local h is (tf - t) / steps.

	local c1 is list(0, 1 / 2, 1 / 2, 1).
    local c2 is list(1 / 6, 2 / 6, 2 / 6, 1 / 6).

	for iter in range(steps) {

		local df is list().
        for i in range(c1:length) {

			local tm is t + c1[i] * h.
			local ym is deep_copy(y).
			
			if i > 0 {
				for k in range(n) {
					set ym[k] to ym[k] + h * c1[i] * df[i - 1][k].
				}
			}

			df:add(df_function(tm, ym, p)).
		}
		
		set t to t + h.

        for i in range(c1:length) {
			for j in range(n) {
				set y[j] to y[j] + h * c2[i] * df[i][j].
			}
		}
	}

	return list(t, y).
}

// returns solution to initial value problem with n equations using runge kutta method (3rd order)
function rk3 {

    parameter df_function.
	parameter p.
	parameter n.
	parameter t.
	parameter tf.
	parameter yi.
	parameter settings.
	
	local y is deep_copy(yi).
	local steps is max(floor((tf - t) / settings:max_step), 0) + 1.
	local h is (tf - t) / steps.

    local c1 is list(0, 1 / 2, 3 / 4).
    local c2 is list(2 / 9, 1 / 3, 4 / 9).

	for iter in range(steps) {

		local df is list().
        for i in range(c1:length) {

			local tm is t + c1[i] * h.
			local ym is deep_copy(y).
			
			if i > 0 {
				for k in range(n) {
					set ym[k] to ym[k] + h * c1[i] * df[i - 1][k].
				}
			}

			df:add(df_function(tm, ym, p)).
		}
		
		set t to t + h.

        for i in range(c1:length) {
			for j in range(n) {
				set y[j] to y[j] + h * c2[i] * df[i][j].
			}
		}
	}

	return list(t, y).
}

// returns solution to initial value problem with n equations using runge kutta method (2nd order)
function rk2 {

    parameter df_function.
	parameter p.
	parameter n.
	parameter t.
	parameter tf.
	parameter yi.
	parameter settings.
	
	local y is deep_copy(yi).
	local steps is max(floor((tf - t) / settings:max_step), 0) + 1.
	local h is (tf - t) / steps.

    local c1 is list(0, 2 / 3).
    local c2 is list(1 / 4, 3 / 4).

	for iter in range(steps) {

		local df is list().
        for i in range(c1:length) {

			local tm is t + c1[i] * h.
			local ym is deep_copy(y).
			
			if i > 0 {
				for k in range(n) {
					set ym[k] to ym[k] + h * c1[i] * df[i - 1][k].
				}
			}

			df:add(df_function(tm, ym, p)).
		}
		
		set t to t + h.

        for i in range(c1:length) {
			for j in range(n) {
				set y[j] to y[j] + h * c2[i] * df[i][j].
			}
		}
	}

	return list(t, y).
}

// returns arcsin of input if it is out of bounds returns -90 or 90 degrees
function safe_arcsin {
    parameter a.

    if a > 1 {
        return 90.
    }
    else if a < -1 {
        return -90.
    }
    else {
        return arcsin(a).
    }
}

// returns arccos of input if it is out of bounds returns 0 or 180 degrees
function safe_arccos {
    parameter a.

    if a > 1 {
        return 0.
    }
    else if a < -1 {
        return 180.
    }
    else {
        return arccos(a).
    }
}
