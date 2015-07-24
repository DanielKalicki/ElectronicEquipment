if (a == 2)
{ 
	if(b == 3)	/* fail Rule 3.5a */
	{
		fun(a,b,c); /* fail Rule 3.5b */
		fun(a, b, c);
	}
	else(b == 3)	/* fail Rule 3.5a */
	{
		foo(a, b, c, d,e,f); /* fail Rule 3.5b */
		c = a / b;
		c|=a+b;
		a &= b / b;
	}
	while(b == 3) /* fail Rule 3.5a */
	{
		a++;
		a ++; /* fail Rule 3.5c */
		--b;
		-- b; /* fail Rule 3.5c */
		!a;
		~a;
		! a;  /* fail Rule 3.5c */
	}
}
