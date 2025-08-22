/*
 * add_calib_maintainer.h
 *
 * Created: 22.08.2025 15:27:32
 *  Author: haasr
 */ 

#pragma once

# include "adc_calib.h"
	
template<typename T>
class AdcCalibMaintainer {
	public:
	AdcCalibMaintainer(uint16_t adc_1, T val_1, uint16_t adc_2, T val_2)
	: adc_1_(adc_1<adc_2?adc_1:adc_2), 
	val_1_(adc_1<adc_2?val_1:val_2), 
	adc_2_(adc_1<adc_2?adc_2:adc_1), 
	val_2_(adc_1<adc_2?val_2:val_1) {
		if(adc_2_== adc_1_) return;
		slope_ =(val_2_ - val_1_)/(adc_2_-adc_1_);
		offset_ = val_1_ - slope_*adc_1_;
	}
	
	AdcCalib<T> Create(uint16_t adc_max= 1023){
		return AdcCalib<T>(adc_max, slope_, offset_);
	}
	
	void ReCalculateStraightLine(){
		if(adc_2_== adc_1_) return;
		slope_ = (val_2_ - val_1_)/(adc_2_ - adc_1_);
		offset_ = val_2_ - slope_*adc_2_;
	}

	void ApplyCalibration(AdcCalib<T>& adcCalib){
		adcCalib.setSlope(slope_);
		adcCalib.setOffset(offset_);
	}
	
	void ReCalibrateLower(uint16_t adc_1, T val_1){
		adc_1_ = adc_1;
		val_1_ = val_1;
		ReCalculateStraightLine();
	}

	void ReCalibrateUpper(uint16_t adc_2, T val_2){
		adc_2_ = adc_2;
		val_2_ = val_2;
		ReCalculateStraightLine();
	}
	
	private:
	// Point 1 == lower ADC value
	uint16_t adc_1_;
	T val_1_;
	// Point 2 == upper ADC value
	uint16_t adc_2_;
	T val_2_;
	T slope_;
	T offset_;
};
