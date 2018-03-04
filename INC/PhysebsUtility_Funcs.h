#pragma once

#include <vector>
#include <sstream>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include "tinyxml2\tinyxml2.h"

namespace Physebs {
	/**
	*	@brief Return the smaller of two values, or the first value if both are equal.
	*	@param a_val1 is the first value.
	*	@param a_val2 is the second value.
	*	@return const reference to smallest value.
	*/
	template<class T>
	const T& Min(const T& a_val1, const T& a_val2) {
		return !(a_val2 < a_val1) ? a_val1 : a_val2;		// Return smaller value or val1 if val1 and val2 are equal
	}

	/**
	*	@brief Return the smaller of two values, or the first value if both are equal.
	*	@param a_val1 is the first value.
	*	@param a_val2 is the second value.
	*	@return const reference to smallest value.
	*/
	template<class T>
	const T& Max(const T& a_val1, const T& a_val2) {
		return !(a_val2 > a_val1) ? a_val1 : a_val2;
	}

	/**
	*	@brief Constrain given value between a range.
	*	@param a_val is the value to constrain.
	*	@param a_upper is the upper range.
	*	@param a_lower is the lower range.
	*	@return const reference to constrained value.
	*/
	template<class T>
	const T& Clamp(const T& a_val, const T& a_upper, const T& a_lower) {
		return Min<T>(Max<T>(a_val, a_lower), a_upper);
	}

	/// Credit to: http://code.runnable.com/VHb0hWMZp-ws1gAr/splitting-a-string-into-a-vector-for-c%2B%2B
	/**
	*	@brief Take in a string and split it by a given delimiter
	*	@param a_str is the string to split.
	*	@param a_delim is the char that defines the split.
	*	@return vector of split strings.
	*/
	static std::vector<std::string> StringToVector(std::string a_str, char a_delim) {
		std::vector<std::string> split;
		// Convert to string stream to allow for easy splitting
		std::stringstream ss(a_str);
		std::string tok;				/*Token (split part of string)*/

		// Use tokens and getline function to split and add to vector
		while (std::getline(ss, tok, a_delim)) {
			split.push_back(tok);
		}

		return split;
	}

	/**
	*	@brief Convert a const char* into a glm vec2.
	*	NOTE: a_str MUST be formatted where there are 2 values separated by commas with no spaces.
	*	e.g. "10.003,10.223"
	*	@param a_str is the string to process into a vec2.
	*	@param a_vec2Ref is the vec3 to modify with converted data.
	*	@return TRUE: conversion was successful | FALSE: conversion was not successful
	*/
	static bool StringToGLMVec2(const char* a_str, glm::vec2& a_vec2Ref) {
		std::string	text = a_str;
		auto textSplit = StringToVector(text, ',');	// Split string to get individual float values
		float x, y;									// CThis function only modifies the first two elements

		// Use TinyXML2 to convert from string to float and return early if conversion fails
		bool b_converted;

		b_converted = tinyxml2::XMLUtil::ToFloat(textSplit[0].c_str(), &x);
		if (!b_converted) return false;

		b_converted = tinyxml2::XMLUtil::ToFloat(textSplit[1].c_str(), &y);
		if (!b_converted) return false;

		// Modify vec3 reference with converted variables
		a_vec2Ref.x = x;
		a_vec2Ref.y = y;

		// Full conversion completed
		return true;
	}

	/**
	*	@brief Convert a const char* into a glm vec3.
	*	NOTE: a_str MUST be formatted where there are 3 values separated by commas with no spaces.
	*	e.g. "10.003,10.223,0.001"
	*	@param a_str is the string to process into a vec3.
	*	@param a_vec3Ref is the vec3 to modify with converted data.
	*	@return TRUE: conversion was successful | FALSE: conversion was not successful 
	*/
	static bool StringToGLMVec3(const char* a_str, glm::vec3& a_vec3Ref) {
		std::string	text = a_str;
		auto textSplit = StringToVector(text, ',');	// Split string to get individual float values
		float z;									// This function only converts the third element

		// Use TinyXML2 to convert from string to float and return early if conversion fails
		bool b_converted;

		// Call vec2 conversion function with converted vec2 reference for first two elements
		glm::vec2 vec2Ref(a_vec3Ref);
		b_converted = StringToGLMVec2(a_str, vec2Ref);
		if (!b_converted) return false;

		b_converted = tinyxml2::XMLUtil::ToFloat(textSplit[2].c_str(), &z);
		if (!b_converted) return false;
		
		// Modify vec3 reference with converted variables
		a_vec3Ref.x = vec2Ref.x;
		a_vec3Ref.y = vec2Ref.y;
		a_vec3Ref.z = z;

		// Full conversion completed
		return true;
	}

	/**
	*	@brief Convert a const char* into a glm vec4.
	*	NOTE: a_str MUST be formatted where there are 4 values separated by commas with no spaces.
	*	e.g. "10.003,10.223,0.001,2.003"
	*	@param a_str is the string to process into a vec4.
	*	@param a_vec4Ref is the vec3 to modify with converted data.
	*	@return TRUE: conversion was successful | FALSE: conversion was not successful
	*/
	static bool StringToGLMVec4(const char* a_str, glm::vec4& a_vec4Ref) {
		std::string	text = a_str;
		auto textSplit = StringToVector(text, ',');	// Split string to get individual float values
		float w;									// This function only converts the fourth element

		// Call vec3 conversion function with converted vec4 reference for first three elements
		bool b_converted;

		glm::vec3 vec3Ref(a_vec4Ref);
		b_converted = StringToGLMVec3(a_str, vec3Ref);
		if (!b_converted) return false;

		// Use TinyXML2 to convert from string to float and return if conversion fails
		b_converted = tinyxml2::XMLUtil::ToFloat(textSplit[3].c_str(), &w);
		if (!b_converted) return false;

		// Modify vec4 reference with converted variables
		a_vec4Ref.x = vec3Ref.x;
		a_vec4Ref.y = vec3Ref.y;
		a_vec4Ref.z = vec3Ref.z;
		a_vec4Ref.w = w;

		// Full conversion completed
		return true;
	}

}