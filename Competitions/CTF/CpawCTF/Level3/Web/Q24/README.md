# Q24.[Web]Baby's SQLi - Stage 2-

## How to resolve?

At first, you must information about login page from [Level2/Web/Q22](../../../Level2/Web/Q22).
Then you will get user name `porisuteru` but you don't know exact table name.
You have to get table name.
How to get all table names?

|RDBMS|Statement|
|---|---|
|mysql|show tables;|
|postgresql|SELECT relname AS table_name FROM pg_stat_user_tables;|
|oracle|SELECT table_name FROM user_tables;|
|SQLite3|select name from sqlite_master where type='table';|
|SQL Server|select * from sys.objects;|

Watch your step!!
`@` is special character.
So you must use `"` for table name when use it.

However, you find that there is no any user information.
It means that login page does not refer database in spite of using SQL.
So you can use **SQL Injection**.
For example, `' OR 1=1--`.

In other words?