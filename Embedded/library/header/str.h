/************************************************************
 * File:    str.h                                           *
 * Author:  Dr.Santi Nuratch                                *
 *          Embedded Computing and Control Laboratory       *
 * Update:  06 July 2017, 10.06 PM                          *
 ************************************************************/


#ifndef __STR_H__
#define	__STR_H__
    
   
    
    char* str_copy(char *src, char *dst);
    char* str_raw_copy(char *src, int length, char *dst);
    

    int str_first_char(char *str, char *c) ;
    int str_last_char(char *str, char *c); 
    
    char *str_trim_left(char *src, char *out);
    char *str_trim_right(char *src, char *out);
    
    char *str_remove_char(char *str, char *out, char c);
    char *str_replace_char(char *str, char *out, char org, char rpc );
    
    
    
    
    int str_length(const char *str);
    
    int str_index_of_first_token(const char *str, const char *token);
    int str_raw_index_of_first_token(const char *str, int length, const char *token);
    
    int str_index_of_last_token(const char *str, const char *token);
    int str_index_of_first_char(const char *str, char c);
    int str_index_of_last_char(const char *str, char c);
    char *str_sub_string(const char *src, int index, int length, char *out);
    char *str_crop_string(const char *src, int start, int stop, char *out);
    
    
    // converter
    int str_to_int(char *src);
#endif	

