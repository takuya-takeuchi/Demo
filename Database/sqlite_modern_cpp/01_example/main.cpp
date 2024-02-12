#include <iostream>
#include <tuple>

#include <sqlite_modern_cpp.h>

using namespace  sqlite;
using namespace std;

void create_table(database& db)
{
	// executes the query and creates a 'user' table
	db << "create table if not exists user ("
		  "   _id integer primary key autoincrement not null,"
		  "   age int,"
		  "   name text,"
		  "   weight real"
		  ");";
}

void insert_new_user(database& db, const std::tuple<int, const char16_t*, double>& user)
{
	// inserts a new user record.
	// binds the fields to '?' .
	// note that only types allowed for bindings are :
	//      int ,long, long long, float, double
	//      string , u16string
	// sqlite3 only supports utf8 and utf16 strings, you should use std::string for utf8 and std::u16string for utf16.
	// note that u"my text" is a utf16 string literal of type char16_t * .
	db << "insert into user (age,name,weight) values (?,?,?);"
	   << std::get<0>(user)
	   << std::get<1>(user)
	   << std::get<2>(user);
}

void insert_new_user_utf16(database& db, const std::tuple<int, const std::string, double>& user)
{
	db << u"insert into user (age,name,weight) values (?,?,?);" // utf16 query string
	   << std::get<0>(user)
	   << std::get<1>(user)
	   << std::get<2>(user);
}

template <class T>
T select_count_from_user(database& db)
{
	// selects the count(*) from user table
	// note that you can extract a single culumn single row result only to : int,long,long,float,double,string,u16string
	T count;
	db << "select count(*) from user" >> count;
	return count;
}

void enumerate(database& db, const std::function<void(int, std::string, double)>& function)
{
	// slects from user table on a condition ( age > 18 ) and executes
	// the lambda for each row returned .
	db << "select age,name,weight from user where age > ? ;"
	   << 18
	   >> function;
}

const std::tuple<int, std::string> select_from_user(database& db)
{
	int age;
	string name;

	// you can also extract multiple column rows
	db << "select age, name from user where _id=1;" >> std::tie(age, name);
	return std::make_tuple(age, name);
}

int main()
{
	try
	{
		// creates a database file 'dbfile.db' if it does not exists.
		database db("dbfile.db");

		create_table(db);
		
		std::tuple<int, const char16_t*, double> bob = std::make_tuple(20, u"bob", 83.25);
		insert_new_user(db, bob);

		int age = 21;
		float weight = 68.5;
		string name = "jack";
		std::tuple<int, std::string, double> jack = std::make_tuple(age, name, weight);
		insert_new_user_utf16(db, jack);

		std::cout << "The new record got assigned id " << db.last_insert_rowid() << std::endl;
	
    	const auto lambda = [](int age, std::string name, double weight)
		{
			std::cout << age << ' ' << name << ' ' << weight << std::endl;
		};
		enumerate(db, lambda);

		const auto count = select_count_from_user<int>(db);
		std::cout << "count : " << count << std::endl;

		const auto user = select_from_user(db);
		std::cout << "Age = " << std::get<0>(user) << ", name = " << std::get<1>(user) << std::endl;

		const auto str_count = select_count_from_user<std::string>(db);
		std::cout << "scount : " << str_count << std::endl;
	}
	catch (exception& e)
	{
		std::cout << e.what() << std::endl;
	}
}