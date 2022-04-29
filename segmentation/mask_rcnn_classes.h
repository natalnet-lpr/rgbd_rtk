#pragma once

enum class MaskRcnnClass : uint8_t
{
  Person = 0,
  Bicycle,
  Car,
  Motorcycle,
  Airplane,
  Bus,
  Train,
  Truck,
  Boat,
  TrafficLight,
  FireHydrant,
  StopSign,
  ParkingMeter,
  Bench,
  Bird,
  Cat,
  Dog,
  Horse,
  Sheep,
  Cow,
  Elephant,
  Bear,
  Zebra,
  Giraffe,
  Backpack,
  Umbrella,
  Handbag,
  Tie,
  Suitcase,
  Frisbee,
  Skis,
  Snowboard,
  SportsBall,
  Kite,
  BaseballBat,
  BaseballGlove,
  Skateboard,
  Surfboard,
  TennisRacket,
  Bottle,
  WineGlass,
  Cup,
  Fork,
  Knife,
  Spoon,
  Bowl,
  Banana,
  Apple,
  Sandwich,
  Orange,
  Broccoli,
  Carrot,
  HotDog,
  Pizza,
  Donut,
  Cake,
  Chair,
  Couch,
  PottedPlant,
  Bed,
  DiningTable,
  Toilet,
  Tv,
  Laptop,
  Mouse,
  Remote,
  Keyboard,
  CellPhone,
  Microwave,
  Oven,
  Toaster,
  Sink,
  Refrigerator,
  Book,
  Clock,
  Vase,
  Scissors,
  TeddyBear,
  HairDrier,
  Toothbrush,

  InvalidClass = 254

};

inline DnnObjectClass parseMaskRcnnClass(const MaskRcnnClass &mask_rcnn_class)
{

  std::function<uint8_t(MaskRcnnClass)> toUint8 = [](MaskRcnnClass mask_rcnn_class)
  {
    return static_cast<uint8_t>(mask_rcnn_class);
  };

  DnnObjectClass dnn_class("Invalid", toUint8(MaskRcnnClass::InvalidClass));

  switch (mask_rcnn_class)
  {
  case MaskRcnnClass::Person:
    dnn_class.class_name = "Person";
    dnn_class.class_id = toUint8(MaskRcnnClass::Person);
    break;
  case MaskRcnnClass::Bicycle:
    dnn_class.class_name = "Bicycle";
    dnn_class.class_id = toUint8(MaskRcnnClass::Bicycle);
    break;
  case MaskRcnnClass::Car:
    dnn_class.class_name = "Car";
    dnn_class.class_id = toUint8(MaskRcnnClass::Car);
    break;
  case MaskRcnnClass::Motorcycle:
    dnn_class.class_name = "Motorcycle";
    dnn_class.class_id = toUint8(MaskRcnnClass::Motorcycle);
    break;
  case MaskRcnnClass::Airplane:
    dnn_class.class_name = "Airplane";
    dnn_class.class_id = toUint8(MaskRcnnClass::Airplane);
    break;
  case MaskRcnnClass::Bus:
    dnn_class.class_name = "Bus";
    dnn_class.class_id = toUint8(MaskRcnnClass::Bus);
    break;
  case MaskRcnnClass::Train:
    dnn_class.class_name = "Train";
    dnn_class.class_id = toUint8(MaskRcnnClass::Train);
    break;
  case MaskRcnnClass::Truck:
    dnn_class.class_name = "Truck";
    dnn_class.class_id = toUint8(MaskRcnnClass::Truck);
    break;
  case MaskRcnnClass::Boat:
    dnn_class.class_name = "Boat";
    dnn_class.class_id = toUint8(MaskRcnnClass::Boat);
    break;
  case MaskRcnnClass::TrafficLight:
    dnn_class.class_name = "TrafficLight";
    dnn_class.class_id = toUint8(MaskRcnnClass::TrafficLight);
    break;
  case MaskRcnnClass::FireHydrant:
    dnn_class.class_name = "FireHydrant";
    dnn_class.class_id = toUint8(MaskRcnnClass::FireHydrant);
    break;
  case MaskRcnnClass::StopSign:
    dnn_class.class_name = "StopSign";
    dnn_class.class_id = toUint8(MaskRcnnClass::StopSign);
    break;
  case MaskRcnnClass::ParkingMeter:
    dnn_class.class_name = "ParkingMeter";
    dnn_class.class_id = toUint8(MaskRcnnClass::ParkingMeter);
    break;
  case MaskRcnnClass::Bench:
    dnn_class.class_name = "Bench";
    dnn_class.class_id = toUint8(MaskRcnnClass::Bench);
    break;
  case MaskRcnnClass::Bird:
    dnn_class.class_name = "Bird";
    dnn_class.class_id = toUint8(MaskRcnnClass::Bird);
    break;
  case MaskRcnnClass::Cat:
    dnn_class.class_name = "Cat";
    dnn_class.class_id = toUint8(MaskRcnnClass::Cat);
    break;
  case MaskRcnnClass::Dog:
    dnn_class.class_name = "Dog";
    dnn_class.class_id = toUint8(MaskRcnnClass::Dog);
    break;
  case MaskRcnnClass::Horse:
    dnn_class.class_name = "Horse";
    dnn_class.class_id = toUint8(MaskRcnnClass::Horse);
    break;
  case MaskRcnnClass::Sheep:
    dnn_class.class_name = "Sheep";
    dnn_class.class_id = toUint8(MaskRcnnClass::Sheep);
    break;
  case MaskRcnnClass::Cow:
    dnn_class.class_name = "Cow";
    dnn_class.class_id = toUint8(MaskRcnnClass::Cow);
    break;
  case MaskRcnnClass::Elephant:
    dnn_class.class_name = "Elephant";
    dnn_class.class_id = toUint8(MaskRcnnClass::Elephant);
    break;
  case MaskRcnnClass::Bear:
    dnn_class.class_name = "Bear";
    dnn_class.class_id = toUint8(MaskRcnnClass::Bear);
    break;
  case MaskRcnnClass::Zebra:
    dnn_class.class_name = "Zebra";
    dnn_class.class_id = toUint8(MaskRcnnClass::Zebra);
    break;
  case MaskRcnnClass::Giraffe:
    dnn_class.class_name = "Giraffe";
    dnn_class.class_id = toUint8(MaskRcnnClass::Giraffe);
    break;
  case MaskRcnnClass::Backpack:
    dnn_class.class_name = "Backpack";
    dnn_class.class_id = toUint8(MaskRcnnClass::Backpack);
    break;
  case MaskRcnnClass::Umbrella:
    dnn_class.class_name = "Umbrella";
    dnn_class.class_id = toUint8(MaskRcnnClass::Umbrella);
    break;
  case MaskRcnnClass::Handbag:
    dnn_class.class_name = "Handbag";
    dnn_class.class_id = toUint8(MaskRcnnClass::Handbag);
    break;
  case MaskRcnnClass::Tie:
    dnn_class.class_name = "Tie";
    dnn_class.class_id = toUint8(MaskRcnnClass::Tie);
    break;
  case MaskRcnnClass::Suitcase:
    dnn_class.class_name = "Suitcase";
    dnn_class.class_id = toUint8(MaskRcnnClass::Suitcase);
    break;
  case MaskRcnnClass::Frisbee:
    dnn_class.class_name = "Frisbee";
    dnn_class.class_id = toUint8(MaskRcnnClass::Frisbee);
    break;
  case MaskRcnnClass::Skis:
    dnn_class.class_name = "Skis";
    dnn_class.class_id = toUint8(MaskRcnnClass::Skis);
    break;
  case MaskRcnnClass::Snowboard:
    dnn_class.class_name = "Snowboard";
    dnn_class.class_id = toUint8(MaskRcnnClass::Snowboard);
    break;
  case MaskRcnnClass::SportsBall:
    dnn_class.class_name = "SportsBall";
    dnn_class.class_id = toUint8(MaskRcnnClass::SportsBall);
    break;
  case MaskRcnnClass::Kite:
    dnn_class.class_name = "Kite";
    dnn_class.class_id = toUint8(MaskRcnnClass::Kite);
    break;
  case MaskRcnnClass::BaseballBat:
    dnn_class.class_name = "BaseballBat";
    dnn_class.class_id = toUint8(MaskRcnnClass::BaseballBat);
    break;
  case MaskRcnnClass::BaseballGlove:
    dnn_class.class_name = "BaseballGlove";
    dnn_class.class_id = toUint8(MaskRcnnClass::BaseballGlove);
    break;
  case MaskRcnnClass::Skateboard:
    dnn_class.class_name = "Skateboard";
    dnn_class.class_id = toUint8(MaskRcnnClass::Skateboard);
    break;
  case MaskRcnnClass::Surfboard:
    dnn_class.class_name = "Surfboard";
    dnn_class.class_id = toUint8(MaskRcnnClass::Surfboard);
    break;
  case MaskRcnnClass::TennisRacket:
    dnn_class.class_name = "TennisRacket";
    dnn_class.class_id = toUint8(MaskRcnnClass::TennisRacket);
    break;
  case MaskRcnnClass::Bottle:
    dnn_class.class_name = "Bottle";
    dnn_class.class_id = toUint8(MaskRcnnClass::Bottle);
    break;
  case MaskRcnnClass::WineGlass:
    dnn_class.class_name = "WineGlass";
    dnn_class.class_id = toUint8(MaskRcnnClass::WineGlass);
    break;
  case MaskRcnnClass::Cup:
    dnn_class.class_name = "Cup";
    dnn_class.class_id = toUint8(MaskRcnnClass::Cup);
    break;
  case MaskRcnnClass::Fork:
    dnn_class.class_name = "Fork";
    dnn_class.class_id = toUint8(MaskRcnnClass::Fork);
    break;
  case MaskRcnnClass::Knife:
    dnn_class.class_name = "Knife";
    dnn_class.class_id = toUint8(MaskRcnnClass::Knife);
    break;
  case MaskRcnnClass::Spoon:
    dnn_class.class_name = "Spoon";
    dnn_class.class_id = toUint8(MaskRcnnClass::Spoon);
    break;
  case MaskRcnnClass::Bowl:
    dnn_class.class_name = "Bowl";
    dnn_class.class_id = toUint8(MaskRcnnClass::Bowl);
    break;
  case MaskRcnnClass::Banana:
    dnn_class.class_name = "Banana";
    dnn_class.class_id = toUint8(MaskRcnnClass::Banana);
    break;
  case MaskRcnnClass::Apple:
    dnn_class.class_name = "Apple";
    dnn_class.class_id = toUint8(MaskRcnnClass::Apple);
    break;
  case MaskRcnnClass::Sandwich:
    dnn_class.class_name = "Sandwich";
    dnn_class.class_id = toUint8(MaskRcnnClass::Sandwich);
    break;
  case MaskRcnnClass::Orange:
    dnn_class.class_name = "Orange";
    dnn_class.class_id = toUint8(MaskRcnnClass::Orange);
    break;
  case MaskRcnnClass::Broccoli:
    dnn_class.class_name = "Broccoli";
    dnn_class.class_id = toUint8(MaskRcnnClass::Broccoli);
    break;
  case MaskRcnnClass::Carrot:
    dnn_class.class_name = "Carrot";
    dnn_class.class_id = toUint8(MaskRcnnClass::Carrot);
    break;
  case MaskRcnnClass::HotDog:
    dnn_class.class_name = "HotDog";
    dnn_class.class_id = toUint8(MaskRcnnClass::HotDog);
    break;
  case MaskRcnnClass::Pizza:
    dnn_class.class_name = "Pizza";
    dnn_class.class_id = toUint8(MaskRcnnClass::Pizza);
    break;
  case MaskRcnnClass::Donut:
    dnn_class.class_name = "Donut";
    dnn_class.class_id = toUint8(MaskRcnnClass::Donut);
    break;
  case MaskRcnnClass::Cake:
    dnn_class.class_name = "Cake";
    dnn_class.class_id = toUint8(MaskRcnnClass::Cake);
    break;
  case MaskRcnnClass::Chair:
    dnn_class.class_name = "Chair";
    dnn_class.class_id = toUint8(MaskRcnnClass::Chair);
    break;
  case MaskRcnnClass::Couch:
    dnn_class.class_name = "Couch";
    dnn_class.class_id = toUint8(MaskRcnnClass::Couch);
    break;
  case MaskRcnnClass::PottedPlant:
    dnn_class.class_name = "PottedPlant";
    dnn_class.class_id = toUint8(MaskRcnnClass::PottedPlant);
    break;
  case MaskRcnnClass::Bed:
    dnn_class.class_name = "Bed";
    dnn_class.class_id = toUint8(MaskRcnnClass::Bed);
    break;
  case MaskRcnnClass::DiningTable:
    dnn_class.class_name = "DiningTable";
    dnn_class.class_id = toUint8(MaskRcnnClass::DiningTable);
    break;
  case MaskRcnnClass::Toilet:
    dnn_class.class_name = "Toilet";
    dnn_class.class_id = toUint8(MaskRcnnClass::Toilet);
    break;
  case MaskRcnnClass::Tv:
    dnn_class.class_name = "Tv";
    dnn_class.class_id = toUint8(MaskRcnnClass::Tv);
    break;
  case MaskRcnnClass::Laptop:
    dnn_class.class_name = "Laptop";
    dnn_class.class_id = toUint8(MaskRcnnClass::Laptop);
    break;
  case MaskRcnnClass::Mouse:
    dnn_class.class_name = "Mouse";
    dnn_class.class_id = toUint8(MaskRcnnClass::Mouse);
    break;
  case MaskRcnnClass::Remote:
    dnn_class.class_name = "Remote";
    dnn_class.class_id = toUint8(MaskRcnnClass::Remote);
    break;
  case MaskRcnnClass::Keyboard:
    dnn_class.class_name = "Keyboard";
    dnn_class.class_id = toUint8(MaskRcnnClass::Keyboard);
    break;
  case MaskRcnnClass::CellPhone:
    dnn_class.class_name = "CellPhone";
    dnn_class.class_id = toUint8(MaskRcnnClass::CellPhone);
    break;
  case MaskRcnnClass::Microwave:
    dnn_class.class_name = "Microwave";
    dnn_class.class_id = toUint8(MaskRcnnClass::Microwave);
    break;
  case MaskRcnnClass::Oven:
    dnn_class.class_name = "Oven";
    dnn_class.class_id = toUint8(MaskRcnnClass::Oven);
    break;
  case MaskRcnnClass::Toaster:
    dnn_class.class_name = "Toaster";
    dnn_class.class_id = toUint8(MaskRcnnClass::Toaster);
    break;
  case MaskRcnnClass::Sink:
    dnn_class.class_name = "Sink";
    dnn_class.class_id = toUint8(MaskRcnnClass::Sink);
    break;
  case MaskRcnnClass::Refrigerator:
    dnn_class.class_name = "Refrigerator";
    dnn_class.class_id = toUint8(MaskRcnnClass::Refrigerator);
    break;
  case MaskRcnnClass::Book:
    dnn_class.class_name = "Book";
    dnn_class.class_id = toUint8(MaskRcnnClass::Book);
    break;
  case MaskRcnnClass::Clock:
    dnn_class.class_name = "Clock";
    dnn_class.class_id = toUint8(MaskRcnnClass::Clock);
    break;
  case MaskRcnnClass::Vase:
    dnn_class.class_name = "Vase";
    dnn_class.class_id = toUint8(MaskRcnnClass::Vase);
    break;
  case MaskRcnnClass::Scissors:
    dnn_class.class_name = "Scissors";
    dnn_class.class_id = toUint8(MaskRcnnClass::Scissors);
    break;
  case MaskRcnnClass::TeddyBear:
    dnn_class.class_name = "TeddyBear";
    dnn_class.class_id = toUint8(MaskRcnnClass::TeddyBear);
    break;
  case MaskRcnnClass::HairDrier:
    dnn_class.class_name = "HairDrier";
    dnn_class.class_id = toUint8(MaskRcnnClass::HairDrier);
    break;
  case MaskRcnnClass::Toothbrush:
    dnn_class.class_name = "Toothbrush";
    dnn_class.class_id = toUint8(MaskRcnnClass::Toothbrush);
    break;
  }

  return dnn_class;
}