/*
 * sp6KX_rtc.c
 *
 *  Created on: 18 de may. de 2017
 *      Author: pablo
 *
 *  El RTC lo implemento como un contador de 32bits que cuenta c/1s
 *  Dado que este micro no puede guardarlo con baterybackup, cuando
 *  se inicializa, lo seteo en 1/1/2000.
 *  Fijo la interrupcion de RTC para que cuente de a 1s y lo incremente.
 *
 *  La librerira avr-glibc no tiene implementadas las funciones de time
 *  ya que son dependientes del S.O.
 *  Por esto, compilo los fuentes de las funciones que uso.
 *
 */

#include "sp6K_rtc.h"
#include "../xmega01.h"

static uint32_t rtc_counter;

struct tm       __tm_store;
long            __utc_offset;
int             (*__dst_ptr) (const time_t *, int32_t *);

void gmtime_r(const time_t * timer, struct tm * timeptr);
unsigned char is_leap_year(int year);
time_t mk_gmtime(const struct tm * timeptr);
struct tm *gmtime(const time_t * timeptr);
void localtime_r(const time_t * timer, struct tm * timeptr);
time_t mktime(struct tm * timeptr);

void RTC_test(void)
{

time_t ret;
struct tm info;
char buffer[80];

 info.tm_year = 2017 - 1900;
 info.tm_mon = 5 - 1;
 info.tm_mday = 24;
 info.tm_hour = 12;
 info.tm_min = 38;
 info.tm_sec = 1;
 info.tm_isdst = -1;

 ret = mktime(&info);

 snprintf_P( debug_printfBuff,CHAR128,PSTR("RTC count=%lu\r\n\0"),ret );
 CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );


 memcpy( &info, gmtime(&ret), sizeof(info));

 snprintf_P( debug_printfBuff,CHAR128,PSTR("ANIO=%d,MES=%d,DIA=%d,hh=%d,mm=%d,ss=%d\r\n\0"),info.tm_yday,info.tm_mon, info.tm_mday, info.tm_hour, info.tm_min, info.tm_sec );
 CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );

}
//------------------------------------------------------------------------------------
void RTC_init(void)
{
/* Este modelo de micro xmaga265A3U solo tiene un RTC de 16 bits.
 * El modelo A3Bu tiene uno de 32 bits con soporte de energia desde
 * una  bateria externa.
 */
	// OSCILADOR:
	// Prendo el oscilador de 32K
	OSC.CTRL |= OSC_RC32KEN_bm;
	// y espero que se estabilize
	do {
	} while ( ( OSC.STATUS & OSC_RC32KRDY_bm ) == 0);


	// CLOCK:
	// Seteo el clock de este oscilador como la fuente del RTC.
	// Es un reloj de 1024 derivado del interno de 32768.
	CLK.RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;

	// RTC:
	// Espero que el RTC no este busy
	do {
	} while ( ( RTC.STATUS & RTC_SYNCBUSY_bm ) );

	// Voy a interrumpir por overflow c/1023
	RTC.PER = RTC_CYCLES_1S - 1;	// Periodo 1023
	RTC.CNT = 0;
	RTC.COMP = 0;
	RTC.CTRL =  RTC_PRESCALER_DIV1_gc;	// Arranco y no uso prescaler.

	portENTER_CRITICAL();
	// Seteo la interrupcion
	RTC.INTCTRL = 0x01;

	/* Enable interrupts. */
	//PMIC.CTRL |= PMIC_LOLVLEN_bm;

	portEXIT_CRITICAL();

}
//------------------------------------------------------------------------------------
void RTC_write_time(struct tm *date )
{
	// Recibe una estructura tm y actualiza el RTC

	rtc_counter = mktime(date);
}
//------------------------------------------------------------------------------------
void RTC_read_time (struct tm *date )
{
	// Lee el RTC y lo convierte a una estructura tm

	date = gmtime(&rtc_counter);

}
//------------------------------------------------------------------------------------
ISR(RTC_OVF_vect)
{
	// Interrumpe c/1s.
	rtc_counter++;
}
//------------------------------------------------------------------------------------
// FUNCIONES DE LA LIBRERIA TIME
//------------------------------------------------------------------------------------
struct tm *gmtime(const time_t * timeptr)
{
	gmtime_r(timeptr, &__tm_store);
	return &__tm_store;
}
//------------------------------------------------------------------------------------
void gmtime_r(const time_t * timer, struct tm * timeptr)
{
    int32_t         fract;
    ldiv_t          lresult;
    div_t           result;
    uint16_t        days, n, leapyear, years;

    /* break down timer into whole and fractional parts of 1 day */
    days = *timer / 86400UL;
    fract = *timer % 86400UL;

    /*
            Extract hour, minute, and second from the fractional day
        */
    lresult = ldiv(fract, 60L);
    timeptr->tm_sec = lresult.rem;
    result = div(lresult.quot, 60);
    timeptr->tm_min = result.rem;
    timeptr->tm_hour = result.quot;

    /* Determine day of week ( the epoch was a Saturday ) */
    n = days + SATURDAY;
    n %= 7;
    timeptr->tm_wday = n;

    /*
        * Our epoch year has the property of being at the conjunction of all three 'leap cycles',
        * 4, 100, and 400 years ( though we can ignore the 400 year cycle in this library).
        *
        * Using this property, we can easily 'map' the time stamp into the leap cycles, quickly
        * deriving the year and day of year, along with the fact of whether it is a leap year.
        */

    /* map into a 100 year cycle */
    lresult = ldiv((long) days, 36525L);
    years = 100 * lresult.quot;

    /* map into a 4 year cycle */
    lresult = ldiv(lresult.rem, 1461L);
    years += 4 * lresult.quot;
    days = lresult.rem;
    if (years > 100)
        days++;

    /*
         * 'years' is now at the first year of a 4 year leap cycle, which will always be a leap year,
         * unless it is 100. 'days' is now an index into that cycle.
         */
    leapyear = 1;
    if (years == 100)
        leapyear = 0;

    /* compute length, in days, of first year of this cycle */
    n = 364 + leapyear;

    /*
     * if the number of days remaining is greater than the length of the
     * first year, we make one more division.
     */
    if (days > n) {
        days -= leapyear;
        leapyear = 0;
        result = div(days, 365);
        years += result.quot;
        days = result.rem;
    }
    timeptr->tm_year = 100 + years;
    timeptr->tm_yday = days;

    /*
            Given the year, day of year, and leap year indicator, we can break down the
            month and day of month. If the day of year is less than 59 (or 60 if a leap year), then
            we handle the Jan/Feb month pair as an exception.
        */
    n = 59 + leapyear;
    if (days < n) {
        /* special case: Jan/Feb month pair */
        result = div(days, 31);
        timeptr->tm_mon = result.quot;
        timeptr->tm_mday = result.rem;
    } else {
        /*
            The remaining 10 months form a regular pattern of 31 day months alternating with 30 day
            months, with a 'phase change' between July and August (153 days after March 1).
            We proceed by mapping our position into either March-July or August-December.
            */
        days -= n;
        result = div(days, 153);
        timeptr->tm_mon = 2 + result.quot * 5;

        /* map into a 61 day pair of months */
        result = div(result.rem, 61);
        timeptr->tm_mon += result.quot * 2;

        /* map into a month */
        result = div(result.rem, 31);
        timeptr->tm_mon += result.quot;
        timeptr->tm_mday = result.rem;
    }

    /*
            Cleanup and return
        */
    timeptr->tm_isdst = 0;  /* gmt is never in DST */
    timeptr->tm_mday++; /* tm_mday is 1 based */

}
//------------------------------------------------------------------------------------
time_t mk_gmtime(const struct tm * timeptr)
{

    time_t          ret;
    uint32_t        tmp;
    int             n, m, d, leaps;

    /*
        Determine elapsed whole days since the epoch to the beginning of this year. Since our epoch is
        at a conjunction of the leap cycles, we can do this rather quickly.
        */
    n = timeptr->tm_year - 100;
    leaps = 0;
    if (n) {
        m = n - 1;
        leaps = m / 4;
        leaps -= m / 100;
        leaps++;
    }
    tmp = 365UL * n + leaps;

    /*
                Derive the day of year from month and day of month. We use the pattern of 31 day months
                followed by 30 day months to our advantage, but we must 'special case' Jan/Feb, and
                account for a 'phase change' between July and August (153 days after March 1).
            */
    d = timeptr->tm_mday - 1;   /* tm_mday is one based */

    /* handle Jan/Feb as a special case */
    if (timeptr->tm_mon < 2) {
        if (timeptr->tm_mon)
            d += 31;

    } else {
        n = 59 + is_leap_year(timeptr->tm_year + 1900);
        d += n;
        n = timeptr->tm_mon - MARCH;

        /* account for phase change */
        if (n > (JULY - MARCH))
            d += 153;
        n %= 5;

        /*
         * n is now an index into a group of alternating 31 and 30
         * day months... 61 day pairs.
         */
        m = n / 2;
        m *= 61;
        d += m;

        /*
         * if n is odd, we are in the second half of the
         * month pair
         */
        if (n & 1)
            d += 31;
    }

    /* Add day of year to elapsed days, and convert to seconds */
    tmp += d;
    tmp *= ONE_DAY;
    ret = tmp;

    /* compute 'fractional' day */
    tmp = timeptr->tm_hour;
    tmp *= ONE_HOUR;
    tmp += timeptr->tm_min * 60UL;
    tmp += timeptr->tm_sec;

    ret += tmp;

    return ret;
}
//------------------------------------------------------------------------------------
unsigned char is_leap_year(int year)
{
    div_t           d;

    /* year must be divisible by 4 to be a leap year */
    if (year & 3)
        return 0;

    /* If theres a remainder after division by 100, year is not divisible by 100 or 400 */
    d = div(year, 100);
    if (d.rem)
        return 1;

    /* If the quotient is divisible by 4, then year is divisible by 400 */
    if ((d.quot & 3) == 0)
        return 1;

    return 0;
}
//------------------------------------------------------------------------------------
time_t mktime(struct tm * timeptr)
{
	time_t          ret;

	ret = mk_gmtime(timeptr);

	if (timeptr->tm_isdst < 0) {
		if (__dst_ptr)
			timeptr->tm_isdst = __dst_ptr(&ret, &__utc_offset);
	}
	if (timeptr->tm_isdst > 0)
		ret -= timeptr->tm_isdst;

	ret -= __utc_offset;

	localtime_r(&ret, timeptr);

	return ret;
}
//------------------------------------------------------------------------------------
void localtime_r(const time_t * timer, struct tm * timeptr)
{
	time_t          lt;
	int16_t         dst;

	dst = -1;

	if (__dst_ptr)
		dst = __dst_ptr(timer, &__utc_offset);

	lt = *timer + __utc_offset;

	if (dst > 0)
		lt += dst;

	gmtime_r(&lt, timeptr);
	timeptr->tm_isdst = dst;
}
//------------------------------------------------------------------------------------
