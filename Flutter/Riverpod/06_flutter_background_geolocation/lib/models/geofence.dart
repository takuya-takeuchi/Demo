class GeoFence {
  String name;
  double latitude;
  double longitude;
  double radius;
  bool selected;
  GeoFence(this.name, this.latitude, this.longitude, this.radius, [this.selected = false]);
}
