import 'package:flutter/material.dart';

import 'package:flutter_widget_from_html/flutter_widget_from_html.dart';
import 'package:html/dom.dart' as dom;
import 'package:uuid/uuid.dart';

class HomePage extends StatefulWidget {
  /// Creates an Home page.
  const HomePage({super.key, required this.title});

  final String title;

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  final List<_OrderList> _lists = new List.empty(growable: true);
  final _uuid = Uuid();

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(widget.title),
      ),
      body: Column(
        children: [
          Flexible(
            child: SingleChildScrollView(
              child: HtmlWidget(
                // Do not use 'id' attribute in this html contents!!!
                '''
              <h1>h1 Heading</h1>
              <a href="https://www.google.com/" target="_blank">Visit www.google.com!</a>
              <h2>h2 Heading</h2>
              <a href="https://www.yahoo.com/" target="_blank">Visit www.yahoo.com!</a>
              <h3>h3 Heading</h3>
              <a href="https://www.msn.com/" target="_blank">Visit www.msn.com!</a>
              <p>
                A paragraph with <strong>strong</strong>, <em>emphasized</em>
                and <span style="color: red">colored</span> text.
              </p>
              <ul type="disc">
                <li>Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.</li>
                <li>Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat.</li>
              </ul>
              <ul type="circle">
                <li>Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur.</li>
                <li>Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.</li>
              </ul>
              <ul type="square">
                <li>Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.</li>
                <li>Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat.</li>
              </ul>
              <ol>
                <li>Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua</li>
                <li>Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat.</li>
                <li>Items</li>
                <ol>
                  <li>SubItem1</li>
                  <li>SubItem2</li>
                  <ol>
                    <li>SubItem1-1</li>
                    <li>SubItem2-1</li>
                  </ol>
                  <li>SubItem3</li>
                  <li>SubItem4</li>
                </ol>
              </ol>
              ''',
                customStylesBuilder: (element) {
                  if (element.classes.contains('foo')) {
                    return {'color': 'red'};
                  }
                  return null;
                },

                customWidgetBuilder: (element) {
                  if (element.id.isEmpty) {
                    // // Generate a v1 (time-based) id
                    element.id = _uuid.v1();
                  }

                  switch (element.localName) {
                    case 'h1':
                      return Text(element.innerHtml, style: TextStyle(fontWeight: FontWeight.bold, fontSize: 24));
                    case 'h2':
                      return Text(element.innerHtml, style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16));
                    case 'h3':
                      return Text(element.innerHtml, style: TextStyle(fontWeight: FontWeight.bold, fontSize: 12));
                    case 'li':
                      if (element.parent!.id.isEmpty) {
                        // // Generate a v1 (time-based) id
                        element.parent!.id = _uuid.v1();
                      }

                      var parent = _findElement(_lists, element.parent!.id);
                      if (parent == null) {
                        parent = _OrderList(element.parent!);
                        _lists.add(parent);
                      }

                      parent.children.add(_OrderList(element));

                      const textStyle = TextStyle(fontSize: 12, color: Colors.black);
                      final text = Text(element.innerHtml, style: textStyle);

                      switch (parent.element.localName) {
                        case 'ol':
                          final container = Container(
                            margin: EdgeInsets.only(right: 5),
                            child: Text(parent.children.length.toString() + ".", style: textStyle),
                          );

                          return Transform.translate(
                            offset: Offset(-15, 0),
                            child: Row(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Padding(
                                  padding: EdgeInsets.zero,
                                  child: container,
                                ),
                                Flexible(child: text)
                              ],
                            ),
                          );
                        case 'ul':
                          Container? container;
                          switch (parent.element.attributes['type']) {
                            case 'circle':
                              container = Container(
                                height: 10,
                                width: 10,
                                margin: EdgeInsets.only(right: 5),
                                decoration: BoxDecoration(
                                  color: Colors.white,
                                  shape: BoxShape.circle,
                                  border: Border.all(color: Colors.red, width: 1),
                                ),
                              );
                            case 'disc':
                              container = Container(
                                height: 10,
                                width: 10,
                                margin: EdgeInsets.only(right: 5),
                                decoration: const BoxDecoration(
                                  color: Colors.green,
                                  shape: BoxShape.circle,
                                ),
                              );
                            case 'square':
                              container = Container(
                                height: 10,
                                width: 10,
                                margin: EdgeInsets.only(right: 5),
                                decoration: BoxDecoration(
                                  color: Colors.blue,
                                  shape: BoxShape.rectangle,
                                ),
                              );
                          }

                          if (container == null) {
                            return null;
                          }

                          return Transform.translate(
                            offset: Offset(-15, 0),
                            child: Row(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Padding(
                                  padding: EdgeInsets.only(top: 3),
                                  child: container,
                                ),
                                Flexible(child: text)
                              ],
                            ),
                          );
                        default:
                          return null;
                      }
                    default:
                      return null;
                  }
                },
                onTapUrl: (url) {
                  print('tapped $url');
                  // if return false, open browser. Otherwise, nothing happens.
                  return false;
                },
                renderMode: RenderMode.column,
                textStyle: TextStyle(fontSize: 14),
              ),
            ),
          ),
        ],
      ),
    );
  }

  static _OrderList? _findElement(List<_OrderList> elements, String id) {
    for (var item in elements) {
      if (item.element.id == id) {
        return item;
      }

      final result = _findElement(item.children, id);
      if (result != null) {
        return result;
      }
    }

    return null;
  }
}

class _OrderList {
  final dom.Element element;
  final List<_OrderList> children = new List.empty(growable: true);
  _OrderList(this.element);
}
