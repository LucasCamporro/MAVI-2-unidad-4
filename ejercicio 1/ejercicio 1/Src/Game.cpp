#include "Box2DHelper.h"
#include <iostream>
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <tchar.h>
#include "SFMLRenderer.h"
#include <list>

const float DEGTORAD = 0.0174532925199432957f;
using namespace sf;

// Declaración de la función connectBodies
b2RevoluteJoint* connectBodies(b2World* world, b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor, float lowerAngle, float upperAngle);


class Game {
private:
    // Propiedades de la ventana
    int alto;
    int ancho;
    RenderWindow* wnd;
    Color clearColor;

    // Objetos de box2d
    b2World* phyWorld;

    // tiempo de frame
    float frameTime;
    int fps;

    // Cuerpo de box2d
    b2Body* cubocabezaBody;
    b2Body* cubotorsoBody;
    b2Body* cubobrazoizqBody;
    b2Body* cubobrazodereBody;
    b2Body* cubopiernaizqBody;
    b2Body* cubopiernadereBody;
    b2Body* cubobasecanonBody;
    b2Body* cubocanonBody;
    b2Body* obstaculo1Body;
    b2Body* obstaculo2Body;

public:
    // Constructores, destructores e inicializadores
    Game(int ancho, int alto, std::string titulo);
    void CreateEnemy(int x, int y);
    ~Game(void);
    void InitPhysics();

    // Main game loop
    void Loop();
    void DrawGame();
    void UpdatePhysics();
    void DoEvents();
    void SetZoom();
};

Game::Game(int ancho, int alto, std::string titulo) {
    wnd = new RenderWindow(VideoMode(ancho, alto), titulo);
    wnd->setVisible(true);
    fps = 60;
    wnd->setFramerateLimit(fps);
    frameTime = 1.0f / fps;
    SetZoom();
    InitPhysics();
}

void Game::Loop() {
    while (wnd->isOpen()) {
        wnd->clear(clearColor);
        DoEvents();
        UpdatePhysics();
        DrawGame();
        wnd->display();
    }
}

void Game::UpdatePhysics() {
    phyWorld->Step(frameTime, 8, 8);
    phyWorld->ClearForces();
    phyWorld->DebugDraw();
}

void Game::DrawGame() {
    // Dibujamos el suelo
    sf::RectangleShape groundShape(sf::Vector2f(500, 5));
    groundShape.setFillColor(sf::Color::Red);
    groundShape.setPosition(0, 95);
    wnd->draw(groundShape);

    groundShape.setPosition(0, 0);
    wnd->draw(groundShape);

    sf::RectangleShape groundShape2(sf::Vector2f(5, 500));
    groundShape2.setFillColor(sf::Color::Red);
    groundShape2.setPosition(95, 0);
    wnd->draw(groundShape2);

    groundShape2.setPosition(0, 0);
    wnd->draw(groundShape2);

    // Dibujamos cabeza
    sf::RectangleShape cuboShape(sf::Vector2f(2, 4));
    cuboShape.setFillColor(sf::Color::Blue);
    cuboShape.setOrigin(1, 2);  // Ajustamos el origen al centro del cubo
    cuboShape.setPosition(cubocabezaBody->GetPosition().x, cubocabezaBody->GetPosition().y);
    cuboShape.setRotation(cubocabezaBody->GetAngle() * 180 / b2_pi);
    wnd->draw(cuboShape);

    // Dibujamos torso
    sf::RectangleShape cubo2Shape(sf::Vector2f(6, 10));
    cubo2Shape.setFillColor(sf::Color::Magenta);
    cubo2Shape.setOrigin(3, 5);  // Ajustamos el origen al centro del cubo
    cubo2Shape.setPosition(cubotorsoBody->GetPosition().x, cubotorsoBody->GetPosition().y);
    cubo2Shape.setRotation(cubotorsoBody->GetAngle() * 180 / b2_pi);
    wnd->draw(cubo2Shape);

    // Dibujamos brazo izq
    sf::RectangleShape cubobrazoIzqShape(sf::Vector2f(3, 7));
    cubobrazoIzqShape.setFillColor(sf::Color::Magenta);
    cubobrazoIzqShape.setOrigin(1.5, 3.5);
    cubobrazoIzqShape.setPosition(cubobrazoizqBody->GetPosition().x, cubobrazoizqBody->GetPosition().y);
    cubobrazoIzqShape.setRotation(cubobrazoizqBody->GetAngle() * 180 / b2_pi);
    wnd->draw(cubobrazoIzqShape);

     // Dibujamos brazo izq
    sf::RectangleShape cubobrazoDereShape(sf::Vector2f(3, 7));
    cubobrazoDereShape.setFillColor(sf::Color::Magenta);
    cubobrazoDereShape.setOrigin(1.5, 3.5);
    cubobrazoDereShape.setPosition(cubobrazodereBody->GetPosition().x, cubobrazodereBody->GetPosition().y);
    cubobrazoDereShape.setRotation(cubobrazodereBody->GetAngle() * 180 / b2_pi);
    wnd->draw(cubobrazoDereShape);

    // Dibujamos pierna izq
    sf::RectangleShape cubopiernaIzqShape(sf::Vector2f(3, 7));
    cubopiernaIzqShape.setFillColor(sf::Color::Magenta);
    cubopiernaIzqShape.setOrigin(1.5, 3.5);
    cubopiernaIzqShape.setPosition(cubopiernaizqBody->GetPosition().x, cubopiernaizqBody->GetPosition().y);
    cubopiernaIzqShape.setRotation(cubopiernaizqBody->GetAngle() * 180 / b2_pi);
    wnd->draw(cubopiernaIzqShape);

    // Dibujamos pierna dere
    sf::RectangleShape cubopiernaDereShape(sf::Vector2f(3, 7));
    cubopiernaDereShape.setFillColor(sf::Color::Magenta);
    cubopiernaDereShape.setOrigin(1.5, 3.5);
    cubopiernaDereShape.setPosition(cubopiernadereBody->GetPosition().x, cubopiernadereBody->GetPosition().y);
    cubopiernaDereShape.setRotation(cubopiernadereBody->GetAngle() * 180 / b2_pi);
    wnd->draw(cubopiernaDereShape);

    // Dibujamos base del cañon
    sf::RectangleShape cubobasecanonShape(sf::Vector2f(4, 16));
    cubobasecanonShape.setFillColor(sf::Color::Green);
    cubobasecanonShape.setOrigin(2, 8);
    cubobasecanonShape.setPosition(cubobasecanonBody->GetPosition().x, cubobasecanonBody->GetPosition().y);
    wnd->draw(cubobasecanonShape);

    // Dibujamos el cañon
    sf::RectangleShape cubocanonShape(sf::Vector2f(20, 10));
    cubocanonShape.setFillColor(sf::Color::Green);
    cubocanonShape.setOrigin(10, 4);
    cubocanonShape.setPosition(cubocanonBody->GetPosition().x, cubocanonBody->GetPosition().y);
    cubocanonShape.setRotation(cubocanonBody->GetAngle() * 180 / b2_pi);
    wnd->draw(cubocanonShape);

    // Dibujamos el obstaculo1
    sf::CircleShape obstaculo1Shape(4.0f); // Radio del círculo
    obstaculo1Shape.setFillColor(sf::Color::Red);
    obstaculo1Shape.setOrigin(4.0f, 4.0f); // Ajusta el origen al centro del círculo
    obstaculo1Shape.setPosition(obstaculo1Body->GetPosition().x, obstaculo1Body->GetPosition().y);
    wnd->draw(obstaculo1Shape);

    // Dibujamos el obstaculo2
    sf::RectangleShape obsatculo2Shape(sf::Vector2f(8, 8));
    obsatculo2Shape.setFillColor(sf::Color::Yellow);
    obsatculo2Shape.setOrigin(4, 4);
    obsatculo2Shape.setPosition(obstaculo2Body->GetPosition().x, obstaculo2Body->GetPosition().y);
    wnd->draw(obsatculo2Shape);
}

void Game::DoEvents() {
    Event evt;
    while (wnd->pollEvent(evt)) {
        switch (evt.type) {
        case Event::Closed:
            wnd->close();
            break;
            case sf::Event::MouseButtonPressed:
                if (evt.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2i pixelPos = sf::Mouse::getPosition(*wnd);
                    sf::Vector2f worldPos = wnd->mapPixelToCoords(pixelPos);
                    b2Vec2 mouseWorldPos(worldPos.x, worldPos.y);
                    b2Vec2 direction = mouseWorldPos - cubocanonBody->GetPosition();

                    // Mover el ragdoll a la posición del cubocanonBody
                    b2Vec2 canonPosition = cubocanonBody->GetPosition();
                    cubocabezaBody->SetTransform(canonPosition, cubocabezaBody->GetAngle());
                    cubotorsoBody->SetTransform(canonPosition, cubotorsoBody->GetAngle());
                    cubobrazoizqBody->SetTransform(canonPosition, cubobrazoizqBody->GetAngle());
                    cubobrazodereBody->SetTransform(canonPosition, cubobrazodereBody->GetAngle());
                    cubopiernaizqBody->SetTransform(canonPosition, cubopiernaizqBody->GetAngle());
                    cubopiernadereBody->SetTransform(canonPosition, cubopiernadereBody->GetAngle());

                    // Calcular la magnitud de la fuerza en función de la posición x del mouse
                    float forceMagnitude = (worldPos.x / static_cast<float>(wnd->getSize().x)) * 15000.0f; // Ajustar la escala de la fuerza si es necesario

                    // Aplicar la fuerza al ragdoll en la dirección calculada
                    b2Vec2 force = b2Vec2(direction.x * forceMagnitude, direction.y * forceMagnitude);
                    cubocabezaBody->ApplyForceToCenter(force, true);
                }
        }
            break;
        
    }
     // Obtener la posición del mouse relativa a la ventana
   
    sf::Vector2i pixelPos = sf::Mouse::getPosition(*wnd);
    sf::Vector2f worldPos = wnd->mapPixelToCoords(pixelPos);

    // Convertir la posición a coordenadas de Box2D
    b2Vec2 mouseWorldPos(worldPos.x, worldPos.y);

    // Calcular el ángulo entre el cubocanonBody y el mouse
    b2Vec2 direction = mouseWorldPos - cubocanonBody->GetPosition();
    float angle = atan2(direction.y, direction.x);

    // Establecer la rotación del cubocanonBody
    cubocanonBody->SetTransform(cubocanonBody->GetPosition(), angle);

}

// Definimos el área del mundo que veremos en nuestro juego
// Box2D tiene problemas para simular magnitudes muy grandes
void Game::SetZoom() {
    View camara;
    // Posición del view
    camara.setSize(100.0f, 100.0f);
    camara.setCenter(50.0f, 50.0f);
    wnd->setView(camara); // asignamos la cámara
}

enum Category {
    RAGDOLL = 0x0001,
    CANON = 0x0002,
    // Puedes agregar más categorías si es necesario
};

void Game::InitPhysics() {
    // Inicializamos el mundo con la gravedad por defecto
    phyWorld = new b2World(b2Vec2(0.0f, 9.8f));

    // Creamos un piso y paredes
    b2Body* groundBody = Box2DHelper::CreateRectangularStaticBody(phyWorld, 200, 10);
    groundBody->SetTransform(b2Vec2(0.0f, 100.0f), 0.0f);

    b2Body* groundBody2 = Box2DHelper::CreateRectangularStaticBody(phyWorld, 200, 10);
    groundBody2->SetTransform(b2Vec2(0.0f, 0.0f), 0.0f);

    b2Body* groundBody3 = Box2DHelper::CreateRectangularStaticBody(phyWorld, 10, 200);
    groundBody3->SetTransform(b2Vec2(0.0f, 0.0f), 0.0f);

    b2Body* groundBody4 = Box2DHelper::CreateRectangularStaticBody(phyWorld, 10, 200);
    groundBody4->SetTransform(b2Vec2(100.0f, 0.0f), 0.0f);

    // Creamos la cabeza
    b2BodyDef cuboDefinicion;
    cuboDefinicion.type = b2_dynamicBody;
    cuboDefinicion.position = b2Vec2(50.0f, 75.0f); // Posición del cuerpo
    cubocabezaBody = phyWorld->CreateBody(&cuboDefinicion); // Creamos el cuerpo dinámico
    // Creamos una forma (rectangular en este caso) para el cuerpo
    b2PolygonShape cuboShape;
    cuboShape.SetAsBox(1, 2); // Tamaño del cubo
    // Definimos las propiedades del cuerpo
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &cuboShape; // Forma asociada al cuerpo
    fixtureDef.density = 1.0f; // Densidad
    fixtureDef.friction = 0.5f; // Fricción
    // Añadimos la forma al cuerpo
    fixtureDef.filter.categoryBits = RAGDOLL;
    fixtureDef.filter.maskBits = ~CANON; // Colisiona con todo excepto con el cañón
    cubocabezaBody->CreateFixture(&fixtureDef);

    // Creamos el torso
    b2BodyDef cubo2Definicion;
    cubo2Definicion.type = b2_dynamicBody;
    cubo2Definicion.position.Set(50.0f, 82.0f); // Posición del cuerpo
    cubotorsoBody = phyWorld->CreateBody(&cubo2Definicion); // Creamos el cuerpo dinámico
    // Creamos una forma (rectangular en este caso) para el cuerpo
    b2PolygonShape cubo2Shape;
    cubo2Shape.SetAsBox(3.0f, 5.0f); // Tamaño del cubo
    // Definimos las propiedades del cuerpo
    b2FixtureDef fixture2Def;
    fixture2Def.shape = &cubo2Shape; // Forma asociada al cuerpo
    fixture2Def.density = 0.1f; // Densidad
    fixture2Def.friction = 0.5f; // Fricción
    // Añadimos la forma al cuerpo
    fixture2Def.filter.categoryBits = RAGDOLL;
    fixture2Def.filter.maskBits = ~CANON; // Colisiona con todo excepto con el cañón
    cubotorsoBody->CreateFixture(&fixture2Def);

    // Posicionamiento inicial del brazo izquierdo
    b2BodyDef cubobrazoIzqDefinicion;
    cubobrazoIzqDefinicion.type = b2_dynamicBody;
    cubobrazoIzqDefinicion.position.Set(45.3f, 80.5f); // Posición del brazo izquierdo
    cubobrazoizqBody = phyWorld->CreateBody(&cubobrazoIzqDefinicion); // Creamos el cuerpo dinámico
    // Creamos una forma (rectangular en este caso) para el cuerpo
    b2PolygonShape cubobrazoIzqShape;
    cubobrazoIzqShape.SetAsBox(1.5f, 3.5f); // Tamaño del cubo
    // Definimos las propiedades del cuerpo
    b2FixtureDef fixturebrazoIzqDef;
    fixturebrazoIzqDef.shape = &cubobrazoIzqShape; // Forma asociada al cuerpo
    fixturebrazoIzqDef.density = 0.1f; // Densidad
    fixturebrazoIzqDef.friction = 0.5f; // Fricción
    // Añadimos la forma al cuerpo
    fixturebrazoIzqDef.filter.categoryBits = RAGDOLL;
    fixturebrazoIzqDef.filter.maskBits = ~CANON; // Colisiona con todo excepto con el cañón
    cubobrazoizqBody->CreateFixture(&fixturebrazoIzqDef);

    // Creamos el inicial del brazo derecho
    b2BodyDef cubobrazoDereDefinicion;
    cubobrazoDereDefinicion.type = b2_dynamicBody;
    cubobrazoDereDefinicion.position.Set(54.7f, 80.5f); // Posición del cuerpo
    cubobrazodereBody = phyWorld->CreateBody(&cubobrazoDereDefinicion); // Creamos el cuerpo dinámico
    // Creamos una forma (rectangular en este caso) para el cuerpo
    b2PolygonShape cubobrazoDereShape;
    cubobrazoDereShape.SetAsBox(1.5f, 3.5f); // Tamaño del cubo
    // Definimos las propiedades del cuerpo
    b2FixtureDef fixturebrazoDereDef;
    fixturebrazoDereDef.shape = &cubobrazoDereShape; // Forma asociada al cuerpo
    fixturebrazoDereDef.density = 0.1f; // Densidad
    fixturebrazoDereDef.friction = 0.5f; // Fricción
    // Añadimos la forma al cuerpo
    fixturebrazoDereDef.filter.categoryBits = RAGDOLL;
    fixturebrazoDereDef.filter.maskBits = ~CANON; // Colisiona con todo excepto con el cañón
    cubobrazodereBody->CreateFixture(&fixturebrazoDereDef);

    // Creamos la pierna izq
    b2BodyDef cubopiernaIzqDefinicion;
    cubopiernaIzqDefinicion.type = b2_dynamicBody;
    cubopiernaIzqDefinicion.position.Set(48.0f, 89.0f); // Posición del cuerpo
    cubopiernaizqBody = phyWorld->CreateBody(&cubopiernaIzqDefinicion); // Creamos el cuerpo dinámico
    // Creamos una forma (rectangular en este caso) para el cuerpo
    b2PolygonShape cubopiernaIzqShape;
    cubopiernaIzqShape.SetAsBox(1.5f, 3.5f); // Tamaño del cubo
    // Definimos las propiedades del cuerpo
    b2FixtureDef fixturepiernaIzqDef;
    fixturepiernaIzqDef.shape = &cubopiernaIzqShape; // Forma asociada al cuerpo
    fixturepiernaIzqDef.density = 0.1f; // Densidad
    fixturepiernaIzqDef.friction = 0.5f; // Fricción
    // Añadimos la forma al cuerpo
    fixturepiernaIzqDef.filter.categoryBits = RAGDOLL;
    fixturepiernaIzqDef.filter.maskBits = ~CANON; // Colisiona con todo excepto con el cañón
    cubopiernaizqBody->CreateFixture(&fixturepiernaIzqDef);

    // Creamos la pierna derecha
    b2BodyDef cubopiernaDereDefinicion;
    cubopiernaDereDefinicion.type = b2_dynamicBody;
    cubopiernaDereDefinicion.position.Set(52.0f, 89.0f); // Posición del cuerpo
    cubopiernadereBody = phyWorld->CreateBody(&cubopiernaDereDefinicion); // Creamos el cuerpo dinámico
    // Creamos una forma (rectangular en este caso) para el cuerpo
    b2PolygonShape cubopiernaDereShape;
    cubopiernaDereShape.SetAsBox(1.5f, 3.5f); // Tamaño del cubo
    // Definimos las propiedades del cuerpo
    b2FixtureDef fixturepiernaDereDef;
    fixturepiernaDereDef.shape = &cubopiernaDereShape; // Forma asociada al cuerpo
    fixturepiernaDereDef.density = 0.1f; // Densidad
    fixturepiernaDereDef.friction = 0.5f; // Fricción
    // Añadimos la forma al cuerpo
    fixturepiernaDereDef.filter.categoryBits = RAGDOLL;
    fixturepiernaDereDef.filter.maskBits = ~CANON; // Colisiona con todo excepto con el cañón
    cubopiernadereBody->CreateFixture(&fixturepiernaDereDef);

    // Creamos la base del cañon
    b2BodyDef cubobasecanonDefinicion;
    cubobasecanonDefinicion.type = b2_staticBody;
    cubobasecanonDefinicion.position.Set(10.0f, 87.0f); // Posición del cuerpo
    cubobasecanonBody = phyWorld->CreateBody(&cubobasecanonDefinicion); // Creamos el cuerpo dinámico
    // Creamos una forma (rectangular en este caso) para el cuerpo
    b2PolygonShape cubobasecanonShape;
    cubobasecanonShape.SetAsBox(1.0f, 1.0f); // Tamaño del cubo
    // Definimos las propiedades del cuerpo
    b2FixtureDef fixtureBasecanonDef;
    fixtureBasecanonDef.shape = &cubobasecanonShape; // Forma asociada al cuerpo
    fixtureBasecanonDef.density = 0.0f; // Densidad
    fixtureBasecanonDef.friction = 0.0f; // Fricción
    // Añadimos la forma al cuerpo
    fixtureBasecanonDef.filter.categoryBits = CANON;
    fixtureBasecanonDef.filter.maskBits = ~RAGDOLL; // Colisiona con todo excepto con el ragdoll
    cubobasecanonBody->CreateFixture(&fixtureBasecanonDef);

    // Creamos el cañon
    b2BodyDef cubocanonDefinicion;
    cubocanonDefinicion.type = b2_dynamicBody;
    cubocanonDefinicion.position.Set(20.0f, 81.0f); // Posición del cuerpo
    cubocanonBody = phyWorld->CreateBody(&cubocanonDefinicion); // Creamos el cuerpo dinámico
    // Creamos una forma (rectangular en este caso) para el cuerpo
    b2PolygonShape cubocanonShape;
    cubocanonShape.SetAsBox(2.0f, 2.0f); // Tamaño del cubo
    // Definimos las propiedades del cuerpo
    b2FixtureDef fixtureCanonDef;
    fixtureCanonDef.shape = &cubocanonShape; // Forma asociada al cuerpo
    fixtureCanonDef.density = 1.0f; // Densidad
    fixtureCanonDef.friction = 0.0f; // Fricción
    // Añadimos la forma al cuerpo
    fixtureCanonDef.filter.categoryBits = CANON;
    fixtureCanonDef.filter.maskBits = ~RAGDOLL;
    cubocanonBody->CreateFixture(&fixtureCanonDef);

    // Creamos obtaculo 1
    b2BodyDef obstaculo1Definicion;
    obstaculo1Definicion.type = b2_dynamicBody;
    obstaculo1Definicion.position.Set(65.0f, 75.0f); // Posición del cuerpo
    obstaculo1Body = phyWorld->CreateBody(&obstaculo1Definicion); // Creamos el cuerpo dinámico
    // Creamos una forma circular para el cuerpo
    b2CircleShape obstaculo1Shape;
    obstaculo1Shape.m_radius = 4.0f; // Radio del círculo
    // Definimos las propiedades del cuerpo
    b2FixtureDef fixtureobstaculo1Def;
    fixtureobstaculo1Def.shape = &obstaculo1Shape; // Forma asociada al cuerpo
    fixtureobstaculo1Def.density = 1.0f; // Densidad
    fixtureobstaculo1Def.friction = 0.0f; // Fricción
    // Añadimos la forma al cuerpo
    fixtureobstaculo1Def.filter.categoryBits = RAGDOLL;
    fixtureobstaculo1Def.filter.maskBits = ~CANON;
    obstaculo1Body->CreateFixture(&fixtureobstaculo1Def);

    // Creamos obtaculo 2
    b2BodyDef obstaculo2Definicion;
    obstaculo2Definicion.type = b2_staticBody;
    obstaculo2Definicion.position.Set(65.0f, 81.0f); // Posición del cuerpo
    obstaculo2Body = phyWorld->CreateBody(&obstaculo2Definicion); // Creamos el cuerpo dinámico
    // Creamos una forma (rectangular en este caso) para el cuerpo
    b2PolygonShape obsatculo2Shape;
    obsatculo2Shape.SetAsBox(4.0f, 4.0f); // Tamaño del cubo
    // Definimos las propiedades del cuerpo
    b2FixtureDef fixtureobstaculo2Def;
    fixtureobstaculo2Def.shape = &obsatculo2Shape; // Forma asociada al cuerpo
    fixtureobstaculo2Def.density = 1.0f; // Densidad
    fixtureobstaculo2Def.friction = 0.0f; // Fricción
    // Añadimos la forma al cuerpo
    fixtureobstaculo2Def.filter.categoryBits = RAGDOLL;
    fixtureobstaculo2Def.filter.maskBits = ~CANON;
    obstaculo2Body->CreateFixture(&fixtureobstaculo2Def);

    // Inicializamos el ancla correctamente
    b2Vec2 anchorCabezaTorso(0.0f, -5.0f); // Aseguramos que el ancla esté en la base de la cabeza y en la parte superior del torso
    b2Vec2 anchorBrazoIzqTorso(-1.0f, -5.0f); // Aseguramos que el ancla esté en la base de la cabeza y en la parte superior del torso
    b2Vec2 anchorBrazoDereTorso(2.0f, -5.0f); // Aseguramos que el ancla esté en la base de la cabeza y en la parte superior del torso
    b2Vec2 anchorPiernaIzqTorso(-4.0f, 2.0f); // Aseguramos que el ancla esté en la base de la cabeza y en la parte superior del torso
    b2Vec2 anchorPiernaDereTorso(4.0f, 2.0f); // Aseguramos que el ancla esté en la base de la cabeza y en la parte superior del torso
    b2Vec2 anchorcanon(1.0f, -8.0f); // Aseguramos que el ancla esté en la base de la cabeza y en la parte superior del torso

    // Llamamos a la función connectBodies
    connectBodies(phyWorld, cubotorsoBody, cubocabezaBody, anchorCabezaTorso, -30.0f * DEGTORAD, 30.0f * DEGTORAD);
    connectBodies(phyWorld, cubotorsoBody, cubobrazoizqBody, anchorBrazoIzqTorso, -0.0f * DEGTORAD, 65.0f * DEGTORAD);
    connectBodies(phyWorld, cubotorsoBody, cubobrazodereBody, anchorBrazoDereTorso, -69.0f * DEGTORAD, -2.0f * DEGTORAD);
    connectBodies(phyWorld, cubotorsoBody, cubopiernaizqBody, anchorPiernaIzqTorso, -15.0f * DEGTORAD, 15.0f * DEGTORAD);
    connectBodies(phyWorld, cubotorsoBody, cubopiernadereBody, anchorPiernaDereTorso, -15.0f * DEGTORAD, 15.0f * DEGTORAD);
    connectBodies(phyWorld, cubobasecanonBody, cubocanonBody, anchorcanon, -95.0f * DEGTORAD, 15.0f * DEGTORAD);
}

// Implementación de la función connectBodies
b2RevoluteJoint* connectBodies(b2World* world, b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor, float lowerAngle, float upperAngle) {
    b2RevoluteJointDef jointDef;
    jointDef.Initialize(bodyA, bodyB, bodyA->GetWorldPoint(anchor));
    jointDef.lowerAngle = lowerAngle;
    jointDef.upperAngle = upperAngle;
    jointDef.enableLimit = true;

    return static_cast<b2RevoluteJoint*>(world->CreateJoint(&jointDef));
}

Game::~Game(void) {
    delete wnd;
    delete phyWorld;
}


int _tmain(int argc, _TCHAR* argv[]) {
    Game* Juego;
    Juego = new Game(800, 600, "ventana eje 1");
    Juego->Loop();
    delete Juego;
    return 0;
}
