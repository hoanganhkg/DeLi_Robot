/******************************************************************************
 *                                                                            *
 *  @Project    Dev_Car                                                       *
 *	@File       utils.c                                                       *
 * 	@Author	    xxx                                             *
 *  @Version		V1.0                                                                  *
 *                                                                            *
 ******************************************************************************/

/****************************************************************************** 
 * @brief
 * THESE CONVERTERS ARE ENCODED/DECODED USING LITTLE-ENDIAN
 * LSB in lowest index
 * Ex: uint32_t 0x12345678 --> bytes [78][56][34][12]
 *                             Index  0   1   2   3
 ******************************************************************************/

#include "utils.h"
#include "string.h"
/******************************************************************************
 * @fn     Uint16ToBytes     
 * @brief  
 * @param  
 * @retval None
 ******************************************************************************/
void Uint16ToBytes(uint16_t u, uint8_t *b) {
    b[0] = (u >> 8) & 0xFF; // Byte cao (MSB)
    b[1] = u & 0xFF;        // Byte th?p (LSB)
}


/******************************************************************************
 * @fn     BytesToUint16     
 * @brief  
 * @param  
 * @retval None
 ******************************************************************************/
void BytesToUint16(uint8_t *b, uint16_t *u)
{
    uint8_t *p;    
    // Casting a pointer to uint8_t: least significant byte
    p = (uint8_t*)u;
    *p++ = *b++;
    *p   = *b;
}

/******************************************************************************
 * @fn     Uint32ToBytes     
 * @brief  
 * @param  
 * @retval None
 ******************************************************************************/
void Uint32ToBytes(uint32_t *u, uint8_t *b)
{
    uint8_t *p;
    // Casting a pointer to uint8_t: least significant byte
    p = (uint8_t*)u;
    *b++ = *p++;
    *b++ = *p++;
    *b++ = *p++;
    *b   = *p;  
}

/******************************************************************************
 * @fn     BytesToUint32     
 * @brief  
 * @param  
 * @retval None
 ******************************************************************************/
void BytesToUint32(uint8_t *b, uint32_t *u)
{
    uint8_t *p;    
    // Casting a pointer to uint8_t: least significant byte
    p = (uint8_t*)u;
    *p++ = *b++;
    *p++ = *b++;
    *p++ = *b++;
    *p   = *b; 
}

/******************************************************************************
 * @fn     Int16ToBytes     
 * @brief  
 * @param  
 * @retval None
 ******************************************************************************/
void Int16ToBytes(int16_t *i, uint8_t *b)
{
    uint8_t *p;
    // Casting a pointer to uint8_t: least significant byte
    p = (uint8_t*)i;
    *b++ = *p++;
    *b   = *p;  
}

/******************************************************************************
 * @fn     BytesToInt16     
 * @brief  
 * @param  
 * @retval None
 ******************************************************************************/
void BytesToInt16(uint8_t *b, int16_t *i)
{
    uint8_t *p; 
    // Casting a pointer to uint8_t: least significant byte    
    p = (uint8_t*)i;
    *p++ = *b++;
    *p   = *b;
}

/******************************************************************************
 * @fn     Real32ToBytes     
 * @brief  
 * @param  
 * @retval None
 ******************************************************************************/
void Real32ToBytes(real32_t *r, uint8_t *b)
{
    uint8_t *p;
    // Casting a pointer to uint8_t: least significant byte
    p = (uint8_t*)r;
    *b++ = *p++;
    *b++ = *p++; 
    *b++ = *p++;
    *b   = *p;
}

/******************************************************************************
 * @fn     BytesToReal32     
 * @brief  
 * @param  
 * @retval None
 ******************************************************************************/
void BytesToReal32(uint8_t *b, real32_t *r)
{
    uint8_t *p;
    // Casting a pointer to uint8_t: least significant byte
    p = (uint8_t*)r;   
    *p++ = *b++;
    *p++ = *b++; 
    *p++ = *b++;
    *p   = *b;
}

/******************************************************************************
 * @fn     String_Clear     
 * @brief  clear string data to empty
 * @param  string: string to clear
 * @retval None
 ******************************************************************************/
void String_Clear(uint8_t* string)
{
	uint8_t i = 0;
	while(string[i] != '\0')
	{
		string[i] = 0;
		i++;
	}
	i = 0;
}

/******************************************************************************
 * @fn     String_RemoveCRLF     
 * @brief  return data that removed '\r','\n' or both 
 * @param  stringin: string to remove
           stringout: string after remove
 * @retval None
 ******************************************************************************/
void String_RemoveCRLF(uint8_t* stringin, uint8_t* stringout)
{
	 String_Clear(stringout);
	 while(*stringin != '\0')
	 {
		 if(*stringin == '\r' || *stringin == '\n')
		 {
			 //next
		 }
		 else
		 {
			 *(stringout++) = *stringin;
		 }
		 stringin++;
	 }	
}
void String_RemoveVT(uint8_t* stringin, uint8_t* stringout)
{
	 String_Clear(stringout);
	 while(*stringin != '\0')
	 {
		 if(*stringin == '\r' || *stringin == '\n')
		 {
			 //next
		 }
		 else
		 {
			 *(stringout++) = *stringin;
		 }
		 stringin++;
	 }	
}



/******************************************************************************
 * @fn     String_Copy     
 * @brief  Copy StringIn to StringOut
 * @param  stringin: string input
					 stringout: output string
 * @retval None
 ******************************************************************************/
void String_Copy(uint8_t* stringin, uint8_t* stringout)
{
//	static uint8_t i = 0;
		uint8_t i = 0;
	while(stringout[i] != '\0')
	{
		stringout[i] = 0; //clear old data
		i++;
	}
	i = 0;
	while(stringin[i] !='\0')
	{
		stringout[i] = stringin[i];
		i++;
	}
}

/******************************************************************************
 * @fn     String_Split     
 * @brief  split string with special character
 * @param  stringin: string input
					 character: character to split
					 startpos: position of character follow number of "character";
					 stringout: output string
 * @retval None
 ******************************************************************************/
void String_Split(uint8_t* stringin, uint8_t character, uint8_t startpos, uint8_t* stringout)
{
    uint8_t count = 0;
    uint8_t current_pos = 0;
    uint8_t found = 0;

    // Clear the output string
    memset(stringout, 0, strlen((char*)stringout));

    while (*stringin != '\0')
    {
        if (*stringin == character)
        {
            current_pos++;
            if (current_pos > startpos)
            {
                break;
            }
        }
        else
        {
            if (current_pos == startpos)
            {
                *stringout = *stringin;
                stringout++;
                found = 1;
            }
        }
        stringin++;
    }

    // Null-terminate the output string
    if (found)
    {
        *stringout = '\0';
    }
}

/******************************************************************************
 * @fn     String_Split_By_Lng     
 * @brief  split string with startposition and length 
 * @param  stringin: string input
					 startpos: position of character with count from 0->n;
           length: length of data to read
					 stringout: output string
 * @retval None
 ******************************************************************************/
void String_Split_By_Lng(uint8_t* stringin, uint8_t startpos, uint8_t length, uint8_t* stringout)
{
	uint8_t count = 0;
	while(stringout[count] != '\0')
	{
		stringout[count] = 0; //clear string data
		count++;
	}
	count = 0;
	for(count = 0;count<length;count++)
	{
		stringout[count] = stringin[startpos+count];
	}
}

 /******************************************************************************
 * @fn     String_Merger     
 * @brief  merger string with special character
 * @param  strfirst: string input 1
					 strsecond: string input 2
					 character: character to separate 2 path
					 stringout: output string
 * @retval None
 ******************************************************************************/
void String_Merger(uint8_t* strfirst,uint8_t* strsecond,uint8_t character,uint8_t* stringout)
{
	uint8_t i = 0, index = 0;
	//clear old data
	while(stringout[i] != '\0')
	{
		stringout[i] = 0;
		i++;
	}		
	//merger
	i = 0;
	while(strfirst[i] != '\0')
	{
		stringout[i] = strfirst[i];
		i++;
	}
	index = i;
	i = 0;
	if(character != '\0')
	{
		stringout[index] = character;
		index++;
	}
	while(strsecond[i] != '\0')
	{
		stringout[index+i] = strsecond[i];
		i++;
	}
	i = 0;
	index = 0;
}

/******************************************************************************
 * @fn     String_Special    
 * @brief  add special character into string
 * @param  stringin: input string
					 character: character to add into string
					 stringout: output string
 * @retval None
 ******************************************************************************/
void String_Special(uint8_t* stringin, uint8_t character, uint8_t* stringout)
{
	uint8_t i = 0, index = 0;
	while(stringout[i] != '\0')
	{
		stringout[i] = 0;
		i++;
	}
	i = 0;
	if(character != '\0')
	{
		stringout[index] = character;
		index++;
	}
	while(stringin[i] != '\0')
	{
		stringout[index+i] = stringin[i];
		i++;
	}
	if(character != '\0')
	{
		stringout[index+i] = character;
	}
}
/******************************************************************************
 * @fn     StringCompare     
 * @brief  Compare two string
 * @param  str1: string first
           str2: string second
 * @retval compare code
 ******************************************************************************/
int8_t StringCompare(uint8_t* str1, uint8_t* str2)
{
	uint16_t i = 0;
	do
	{
		if (str1[i] > str2[i])
			return (int8_t)(1);
		else if (str1[i] < str2[i])
			return (int8_t)(-1);
		i++;
	} while (str1[i] != '\0' || str2[i] != '\0');
	return (int8_t)(0);
}

/******************************************************************************
 * @fn     Str2Double    
 * @brief  Convert string to double
 * @param  string
 * @retval double
 ******************************************************************************/
double Str2Double(uint8_t *string)
{
	return atof((char*)string);
}

/******************************************************************************
 * @fn     Str2Double_Dev    
 * @brief  Convert string to double
 * @param  string
 * @retval double
 ******************************************************************************/
double Str2Double_Dev(uint8_t* string)
{
	bool dec = true;
	bool frac = false;
	uint8_t *temp = string;
    uint8_t i = 0, deccount = 0, fraccount = 0;
	double dbnum = 0, decnum = 0, fracnum = 0;
	while(*temp != '\0')
	{
		if(*temp == '.')
		{
			dec = false;
			frac = true;
		}
		else
		{
			if(dec == true)
			{
				deccount++;
			}
			if(frac == true)
			{
				fraccount++;
			}
		}
		temp++;
	}
	temp = string;
	for(i = 0;i<deccount;i++)
	{
		decnum += (double)(*temp - 48)*pow(10.0,deccount-i-1);
		temp++;
	}
	temp++;
	for(i = 0;i<fraccount;i++)
	{
		fracnum  += (double)(*temp - 48)/pow(10.0,i+1);
		temp++;
	}
	dbnum = decnum + fracnum;
	return dbnum;
}

/******************************************************************************
 * @fn     Double2Int    
 * @brief  Convert double to integer
 * @param  double
 * @retval integer
 ******************************************************************************/
int32_t Double2Int(double dbnum)
{
	return (int32_t)dbnum;
}

/******************************************************************************
 * @fn     Int2Str    
 * @brief  Convert integer number to string
 * @param  inum: signed integer
					 string: out string
 * @retval None
 ******************************************************************************/
void Int2Str(int32_t inum, uint8_t* string)
{
	bool sign = false; //possitive
	uint8_t numcount = 0, i = 0, j = 0;
	uint32_t temp = 0, posnum = 0, pownum = 0;
	while( string[i] != '\0')
	{
	  string[i] = 0;  //clear string data
		i++;
	}
	i = 0;
	if(inum>=0)//positive
	{
		posnum = inum;
		sign = false;
	}
	else     //negative
	{
		posnum = -inum;
		sign = true;
		*(string++) = '-';
	}
	temp = posnum;
	while(temp > 0)
	{
		numcount++;
		temp = temp/10; // divide and take decimal
	}
	temp = posnum;
	if(numcount == 0) // ~ inum = 0
	{
		*string = '0';
	}
	else
	{
		for(i=0;i<numcount;i++)
		{
			pownum = 1;
			for(j=i+1;j<numcount;j++)
			{
				pownum = pownum*10;
			}
			*(string++) = temp/pownum + 48;
			temp = temp%pownum;
		}
	}
}

/******************************************************************************
 * @fn     Double2Str    
 * @brief  Convert double number to string
 * @param  dbnum: double number
					 num_of_frac: num of fraction after the '.';
					 string: out string
 * @retval None
 ******************************************************************************/
void Double2Str(double dbnum,uint8_t num_of_frac,uint8_t* string)
{
	int32_t decnum = 0, powfracnum = 0;
	double fracnum = 0;
	uint8_t decstr[20] = {0}, fracstr[20] = {0};
	uint8_t i = 0, id = 0, ifr = 0;
	while(string[i] != '\0')
	{
		string[i] = 0;  //clear string data
		i++;
	}
	i = 0;
	decnum = (int)dbnum;// get decimal
	if(dbnum>=0)
	{
		fracnum = dbnum - decnum;//get fraction
	}
	else
	{
		fracnum = -(dbnum - decnum);//get fraction
	}
	if(dbnum < 0 && dbnum > -1) //ex: -0.xx;
	{
		string[id] = '-';
		id++;
		string[id] = '0';
		id++;
	}
	Int2Str(decnum,decstr);
	powfracnum = (uint32_t)(fracnum*pow(10.0,num_of_frac));//convert fraction to decimal with num of fraction
	Int2Str(powfracnum,fracstr);//convert fraction to string
	while(decstr[id] != '\0')// add decimal string to dest string
	{
		string[id] = decstr[id];
		id++;
	}
	string[id] = '.';//add '.'
	while(fracstr[ifr] != '\0')//add fraction string to dest string
	{
		string[id+1+ifr] = fracstr[ifr];
		ifr++;
	}	
}
//*******************************************************
	
void float2Bytes(float val,int8_t * bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    int8_t temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array,4);
}
void convertFloatToBytes(float value, uint8_t buffer[],int k) {
    // S? d?ng con tr? ki?u uint8_t d? truy c?p t?ng byte c?a gi� tr? float
    uint8_t *ptr = (uint8_t *)&value;

    // Sao ch�p t?ng byte c?a gi� tr? float v�o buffer
    for ( int i=k ; i < 4+k; ++i) {
        buffer[i] = *(ptr + i-k);
    }
}
void parseData(uint8_t *data, size_t len, int16_t *output, size_t *outputLen) {
    uint8_t *tokenStart = NULL;  // Start of the token
    int count = 0;
    int16_t number;
    *outputLen = 0;  // Initialize the output length

    // Skip the "$RQ," part if it's at the beginning
    if (data[0] == '$' && data[1] == 'R' && data[2] == 'Q' && data[3] == ',') {
        data += 4;  // Skip "$RQ,"
        len -= 4;   // Adjust length
    }

    for (size_t i = 0; i < len; i++) {
        if (data[i] == ',' || data[i] == '*' || i == len - 1) {
            // End of a token (either comma or end of data)
            if (tokenStart != NULL) {
                size_t tokenLength = &data[i] - tokenStart;
                uint8_t token[tokenLength + 1];
                for (size_t j = 0; j < tokenLength; j++) {
                    token[j] = tokenStart[j];
                }
                token[tokenLength] = '\0';  // Null-terminate the token
                
                // Convert token to int16_t and store in output array
                number = (int16_t)atoi((char*)token);  // Cast to int16_t
                output[count++] = number;  // Store in the output array
                
                (*outputLen)++;  // Increase output length
                tokenStart = NULL;  // Reset token start for next token
            }
        } else if (tokenStart == NULL) {
            tokenStart = &data[i];  // Mark the start of a token
        }
    }
}