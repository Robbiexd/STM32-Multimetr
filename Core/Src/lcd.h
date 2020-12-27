/*
 * lcd.h
 *
 *  Created on: Oct 9, 2020
 *      Author: Spravce
 */

#ifndef SRC_LCD_H_
#define SRC_LCD_H_

#define LCD_POWER_UP_DELAY	30

#define LCD_FUNCTION_SET					//001 DL N F --
#define LCD_ON_OFF_CONTROL
#define LCD_DISPLAY_CLEAR_SET	0b00000001
#define LCD_ENTRY_MODE_SET					//000001 I/D SH

#define LCD_CONTROL GPIOC
#define LCD_DATA GPIOB
#define LCD_RS GPIO_PIN_0
#define LCD_RW GPIO_PIN_1
#define LCD_EN GPIO_PIN_2
#define LCD_LOW GPIO_PIN_RESET
#define LCD_HIGH GPIO_PIN_SET

#define LCD_tSU1	40
#define LCD_tSU2	80
#define LCD_tW		230
#define LCD_tH1		10

uint16_t

void lcd_delay_ms(const uint16_t cnt)
{
	uint32_t n = ((uint32_t)cnt) * 350;
	while (n--) asm("NOP");	//lcd_wait_us(1000);
}

void lcd_delay_us(const uint16_t cnt)
{
	uint32_t n = ((uint32_t)cnt) / 3;
	while (n--) asm("NOP");	// lcd_wait_ns(1000);
}

void lcd_delay_ns(const uint16_t cnt)
{
	uint32_t n = ((uint32_t)cnt) / 3000;
	while (n--) asm("NOP");	// neco co trva 1ns
}


inline bezznam lcd_write_pin(GPIO_TypeDef* port, bezznam 16bit pin, bezznam 8bit hodn)
{
	HAL_GPIO_WritePin(port, pin, hodn);
}

inline bezznam lcd_write_port(GPIO_TypeDef* port, bezznam 16bit hodn)
{
	//HAL_GPIO_WritePin(port, pin, hodn);
	//port->ODR = hodn;

	HAL_GPIO_WritePin(port,(hodn) & 0xFF,GPIO_PIN_SET);
	HAL_GPIO_WritePin(port,(~hodn) & 0xFF,GPIO_PIN_RESET);
}

bezpatram lcd_cmd(bezznam 8bit cmd)
{
	lcd_write_pin(LCD_CONTROL,LCD_RS, LCD_LOW); //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	lcd_write_pin(LCD_CONTROL,LCD_RW, LCD_LOW);
	lcd_delay_ns(LCD_tSU1);
	lcd_write_pin(LCD_CONTROL,LCD_EN, LCD_HIGH);
	lcd_delay_ns(LCD_tW - LCD_tSU2);
	lcd_write_port(LCD_DATA,cmd);
	lcd_delay_ns(LCD_tSU2);
	lcd_write_pin(LCD_CONTROL,LCD_EN, LCD_LOW);
	lcd_delay_ns(LCD_tH1);

	kdyz (cmd rovno LCD_DISPLAY_CLEAR_SET nebo LCD_RETURN_HOME) lcd_delay_us(1530);
	jinak lcd_delay_us(39);
}

bezpatram lcd_data(bezznam 8bit dt)
{
	lcd_write_pin(LCD_CONTROL,LCD_RS, LCD_HIGH);
	lcd_write_pin(LCD_CONTROL,LCD_RW, LCD_LOW);
	lcd_delay_ns(LCD_tSU1);
	lcd_write_pin(LCD_CONTROL,LCD_EN, LCD_HIGH);
	lcd_delay_ns(LCD_tW - LCD_tSU2);
	lcd_write_port(LCD_DATA,cmd);
	lcd_delay_ns(LCD_tSU2);
	lcd_write_pin(LCD_CONTROL,LCD_EN, LCD_LOW);
	lcd_delay_ns(LCD_tH1);

	lcd_delay_us(42);
}

//zapis retezce					0/1						10-49	char* retez
bezparam lcd_write(bezznam 8bit radek, bezznam 8bit sloupec, retezec_znaku retez)
{
	osetrit radek - bitova maska na nejnizsi bit

	osetrit sloupec - modulo poctem znaku na jeden radek

	kdyz radek ruzny od 0 pricist k sloupec LCD_2ND_ROW // 0x40

	lcd_cmd(LCD_SET_DDRAM_ADDRESS secist sloupec)//nastavit pozici

	v cyklu pres vsechny prvky retezce
		lcd_data(retez[i])

}

//definice vlastniho znaku
bezparam lcd_def_char(bezznam 8bit ascii_code, pole_bezznam_8bit znak)
{//precteme si pozici v DDRAM
	osetrit ascii_code .. od 0 do 7

	ascii_code = ascii_code * 8 (bez nasobeni)

	lcd_cmd(LCD_SET_CGRAM_ADDRESS secist ascii_code)//nastavit pozici

	v cyklu pro 8 prvku v poli znak
		lcd_data(znak[i])

	lcd_cmd(LCD_SET_DDRAM_ADDRESS) //vratime zapis na DDRAM
	//obnovime pozici v DDRAM
}

bezparam lcd_init(bezparam)
{
	//LCD_POWER = LCD_ON; lcd_power_on();

	lcd_delay_ms(LCD_POWER_UP_DELAY);

	lcd_cmd(LCD_FUNCTION_SET);
	lcd_cmd(LCD_ON_OFF_CONTROL);
	lcd_cmd(LCD_DISPLAY_CLEAR_SET);
	lcd_cmd(LCD_ENTRY_MODE_SET);

}

#endif /* SRC_LCD_H_ */
