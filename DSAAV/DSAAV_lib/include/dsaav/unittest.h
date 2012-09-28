/**
 * DSAAV Unit testing framework.
 *
 * (c) 2006 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.2 $
 */

#ifndef _UNIT_TEST_H_
#define _UNIT_TEST_H_

void startTest(const char* name);
void test(const char* name, int pass);
void endTest(void);
void printSummary(void);

#endif
