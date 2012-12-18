/**
 * DSAAV Library - Core Classes.
 *
 * (c) 2006 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.3 $
 */

#ifndef _DSAAV_CORE_H_
#define _DSAAV_CORE_H_

/**
 * Variable length character string.
 */
class String {
  private:
    char* buf;
    int buflen;
  public:
    String(void);
    String(int n);
    String(const char* s);
    String(String* s);
    ~String(void);
    int length(void);
    char get(int p);
    String* substr(int p, int n);
    void copy(String* s);
    void copy(const char* s);
    void append(String* s);
    void append(const char* s);
    void append(char c);
    void erase(int p);
    int equals(String* s);
    int equals(const char* s);
    char* chars(void);
};

/**
 * Linked list item.
 * \internal
 */
struct LinkedListItem {
  LinkedListItem* next;
  // actual object data follows
};

/**
 * List of arbitrary objects.
 */
class List {
  private:
    int items;
    LinkedListItem* head;
    LinkedListItem* tail;
  public:
    List(void);
    ~List(void);
    int count(void);
    BufferPtr get(int i);
    int add(const BufferPtr obj, int len);
    int remove(int i);
    int find(const BufferPtr obj, int len);
    void clear(void);
};

// common functions

double getTime(void);

#endif
