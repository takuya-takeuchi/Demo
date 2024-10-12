import 'package:flutter/material.dart';

import 'package:flutter_widget_from_html/flutter_widget_from_html.dart';

class HomePage extends StatefulWidget {
  /// Creates an Home page.
  const HomePage({super.key, required this.title});

  final String title;

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  @override
  void initState() async {
    super.initState();

    await Future.delayed(Duration(seconds: 5));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(widget.title),
      ),
      body: Center(
        child: HtmlWidget(
          // the first parameter (`html`) is required
          '''
          <h3>Heading</h3>
          <p>
            A paragraph with <strong>strong</strong>, <em>emphasized</em>
            and <span style="color: red">colored</span> text.
          </p>
          ''',

          // all other parameters are optional, a few notable params:
          // specify custom styling for an element
          // see supported inline styling below
          customStylesBuilder: (element) {
            if (element.classes.contains('foo')) {
              return {'color': 'red'};
            }
            return null;
          },

          customWidgetBuilder: (element) {
            // if (element.attributes['foo'] == 'bar') {
            //   return FooBarWidget();
            // }

            // if (element.attributes['fizz'] == 'buzz') {
            //   return InlineCustomWidget(
            //     child: FizzBuzzWidget(),
            //   )
            // }

            return null;
          },
          onTapUrl: (url) => print('tapped $url'),
          renderMode: RenderMode.column,
          textStyle: TextStyle(fontSize: 14),
        ),
      ),
    );
  }
}
