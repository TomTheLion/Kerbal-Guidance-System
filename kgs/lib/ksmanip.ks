@lazyglobal off.

//
// functions for manipulation of kOS data structures
//

// format error string for undefined variable and crash program
function require_variable {

	parameter is_defined.
	parameter var_desc.
	parameter var_name.
	
	if not is_defined {
		crash("Required " + var_desc + " " + char(34) + var_name + char(34) + " not found.").
	}
}

// if key is not in lexicon crash program
function require_key {

	parameter a.
	parameter key.
	parameter message is "".
	
	if not a:haskey(key) {
		crash("Required key " + char(34) + key + char(34) + " not found" + message + ".").
	}
}

// if list of keys are not in lexicon crash program
function require_keys {

	parameter a.
	parameter keys.
	parameter message is "".
	
	for key in keys {
		require_key(a, key, message).
	}
}

// add list of key value pairs from one lexicon to another
function add_keys {
	
	parameter a.
	parameter b.
	parameter keys.
	
	for key in keys {
		require_key(b, key).
		a:add(key, b[key]).
	}
}

// add list of key value pairs from one lexicon to another if they exist
function add_keys_if {
	
	parameter a.
	parameter b.
	parameter keys.
	
	for key in keys {
		if b:haskey(key) {
			a:add(key, b[key]).
		}
	}
}

// add list of key value pairs from one lexicon to another if they exist, if they do not exist add specified pair
function add_keys_if_else {
	
	parameter a.
	parameter b.
	parameter keys.
	parameter values.
	
	for i in range(keys:length) {
		local key is keys[i].
		local value is values[i].
		if b:haskey(key) {
			a:add(key, b[key]).
		}
		else {
			a:add(key, value).
		}		
	}
}

// check if value is in list, return true of false
function in_list {

	parameter a.
	parameter b.

	for i in range(b:length){
		if a = b[i] {
			return true.
		}
	}

	return false.
}

// returns a deep copy of a list
function deep_copy_list {
	
    parameter a.

	local b is list().
	for i in range(a:length) {
		if a[i]:typename = "list" {
			b:add(deep_copy_list(a[i])).
		}
		else if a[i]:typename = "lexicon" {
			b:add(deep_copy_lexicon(a[i])).
		}
		else if a[i]:typename = "vector" {
			b:add(a[i]:vec).
		}
		else {
			b:add(a[i]).
		}
	}	

	return b.
}

// returns a deep copy of a lexicon
function deep_copy_lexicon {
	
    parameter a.

	local b is lexicon().
	for key in a:keys {
		if a[key]:typename = "list" {
			b:add(key, deep_copy_list(a[key])).
		}
		else if a[key]:typename = "lexicon" {
			b:add(key, deep_copy_lexicon(a[key])).
		}
		else if a[key]:typename = "vector" {
			b:add(key, a[key]:vec).
		}
		else {
			b:add(key, a[key]).
		}
	}	

	return b.
}

// returns a deep copy of a list or lexicon
function deep_copy {

	parameter a.
	
	if a:typename = "list" {
		return deep_copy_list(a).
	}
	else if a:typename = "lexicon" {
		return deep_copy_lexicon(a).
	}
}

// convert list to vector starting at specified index
function list_to_vector {

    parameter a.
    parameter n.

    return v(a[n], a[n + 1], a[n + 2]).
}

// converts a vector or a list of vectors and values to a list
function vector_to_list {
	
	parameter a.
	
	local b is list().
	
	if a:typename <> "list" {
		set a to list(a).
	}
	
	for i in range(a:length) {
		if a[i]:typename = "vector" {
			b:add(a[i]:x).
			b:add(a[i]:y).
			b:add(a[i]:z).
		}
		else {
			b:add(a[i]).
		}
	}

	return b.
}

// swap y and z components of a vector
function swap_yz {
    
    parameter a.

    return v(a:x, a:z, a:y).
}

// format number to be displayed with specified precision and add thousands separator
function format_number {
	
	parameter num.
	parameter prec is 2.
	
	set num to "" + round(num, prec).

	if not num:contains(".") and prec <> 0 {
		set num to num + ".".
		for i in range(prec) {
			set num to num + "0".
		}
	}
	else if prec <> 0 {
		for i in range(num:length - num:find(".") - 1 - prec) {
			set num to num + "0".
		}
	}
	
	local offset is 1.
	if prec = 0 {
		set offset to 0.
	}
	
	if num:length - offset - prec > 6 {
		set num to num:insert(num:length - 3 - prec - offset, ",").
		set num to num:insert(num:length - 7 - prec - offset, ",").
	}
	else if num:length - offset - prec > 3 {
		set num to num:insert(num:length - 3 - prec - offset, ",").
	}

	return num.
}

// format number to be displayed as a percentage
function format_percent {
	
	parameter num.
	
	set num to "" + round(100 * num, 1).

	if not num:contains(".") {
		set num to num + ".0%".
	}
	else {
		set num to num + "%".
	}

	return num.
}

// format number to be displayed in scientific notation with specified precision
function format_sci {
	
	parameter num.
	parameter prec is 2.
	
	local exponent is 0.
	if num > 0 {
		set exponent to floor(log10(num)).
	}
	else if num < 0 {
		set exponent to floor(log10(-num)).
	}
	
	return format_number(num / 10 ^ exponent, prec) + "e" + exponent.
}

// format and print string to specified location
function print_string {
	
	parameter string.
	parameter col.
	parameter line.
	parameter size.
	parameter alight_right is true.
	
	if string:length > size {
		set string to string:substring(0, size).
	}

	if alight_right {
		set string to string:padleft(size).
	}
	else {
		set string to string:padright(size).
	}
	
	print string at (col, line).	
}

// print message and crash program
function crash {

	set terminal:width to 120.
	set terminal:height to 60.

	parameter m.
	
	print m.
	print " ".
	print " ".
	print " ".
	print " ".
	print " ".
	set _ to ___crash___.
}