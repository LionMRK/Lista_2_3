#include "test.h"
#include <time.h>


class Lista2 : public Test //você cria a sua classe derivada da classe base Test
{
public:

	Lista2() {

		CreateBox(b2Vec2(0.0, -3.8), b2Vec2(46.0, 1.0), 1.0, 0.5, 0.5, b2_staticBody); //bottom wall

		CreateBox(b2Vec2(0.0, 45.0), b2Vec2(46.0, 1.0), 1.0, 0.5, 0.5, b2_staticBody); // top wall

		CreateBox(b2Vec2(45.0, 20.5), b2Vec2(1.0, 25.5), 1.0, 0.5, 0.5, b2_staticBody); // right wall

		CreateBox(b2Vec2(-45.0, 20.5), b2Vec2(1.0, 25.5), 1.0, 0.5, 0.5, b2_staticBody); //left wall
		

		//Exercicio 1
		/*selected = CreateBox(b2Vec2(-20.0, 4.0), b2Vec2(2.0, 6.0), 1.0, 0.5, 0.5);

		CreateBox(b2Vec2(-10.0, 4.0), b2Vec2(2.0, 6.0), 1.0, 0.5, 0.5);

		CreateBox(b2Vec2(0.0, 4.0), b2Vec2(2.0, 6.0), 1.0, 0.5, 0.5);

		CreateBox(b2Vec2(10.0, 4.0), b2Vec2(2.0, 6.0), 1.0, 0.5, 0.5);

		CreateBox(b2Vec2(20.0, 4.0), b2Vec2(2.0, 6.0), 1.0, 0.5, 0.5);*/

		//Exercicio 2

		//selected = CreateCircle(b2Vec2(-30.0, 4.0), rand() % 5 + 1, 1.0, 0.5, 0.5);


		//Exercicio 3
		selected = CreateBox(b2Vec2(0.0, 4.0), b2Vec2(2.0, 6.0), 1.0, 0.5, 0.5);


		m_world->SetGravity(b2Vec2(0, -10));
		applyingForce = false;
		force.SetZero();
		globalPoint.SetZero();

	}

	void Step(Settings& settings) override
	{
		//Chama o passo da simulação e o algoritmo de rendering
		Test::Step(settings);

		processInputs();

		force.Normalize();
		force *= 5;


		g_debugDraw.DrawPoint(selected->GetWorldCenter(), 5, b2Color(1.0, 0.0, 0.0));
		g_debugDraw.DrawSegment(selected->GetWorldCenter(), selected->GetWorldCenter() + force, b2Color(1.0, 0.0, 0.0));

		b2Vec2 localPoint = b2Vec2(0.0, 0.5);
		globalPoint = b2Vec2(selected->GetWorldPoint(localPoint));

		g_debugDraw.DrawPoint(globalPoint, 5, b2Color(1.0, 1.0, 0.0));

		//show some text in the main screen
		g_debugDraw.DrawString(5, m_textLine, "Exercicios");
		m_textLine += 15;

	}

	static Test* Create()  //a classe Test que instancia um objeto da sua nova classe
						   //o autor da Box2D usa um padrão de projeto chamado Factory
						   //para sua arquitetura de classes
	{

		return new Lista2;
	}

	//Para interagir com o teclado
	void Keyboard(int key) override //esse cabeçalho nao mudam
	{

		switch (key)
		{
		case GLFW_KEY_F:

			applyingForce = true;


				break;

		case GLFW_KEY_N:
			do
			{
				if (selected->GetNext() != NULL)
				{
					selected = selected->GetNext();
				}
				else selected = m_world->GetBodyList();
			} while (selected->GetType() != b2_dynamicBody);
			break;

		case GLFW_KEY_MINUS:

			anguloPraLanca -= 10.0f;

			break;

		case GLFW_KEY_KP_ADD:

			anguloPraLanca += 10.0f;

			break;

		case GLFW_KEY_ENTER:

			force = getVectorComponent(10000, anguloPraLanca);
			selected->ApplyForceToCenter(force, true);

			break;
			
		case GLFW_KEY_W:

			force = getVectorComponent(10000, 90);
			selected->ApplyForce(force, globalPoint, true);

		case GLFW_KEY_D:
			anguloPraLanca = 0.0;
			force = getVectorComponent(10000, anguloPraLanca);
			selected->ApplyForce(force, globalPoint, true);

		case GLFW_KEY_A:
			force = getVectorComponent(5000, 180);
			selected->ApplyForce(force, globalPoint, true);

		}
	}


	b2Body* CreateBox(b2Vec2 pos, b2Vec2 dim, float density, float friction, float restitution, b2BodyType = b2_dynamicBody);
	b2Body* CreateCircle(b2Vec2 pos, float radius, float density, float friction, float restitution, b2BodyType = b2_dynamicBody);
	b2Body* CreateLine(b2Vec2 pos, b2Vec2 inicio, b2Vec2 fim, float density, float friction, float restitution, b2BodyType = b2_dynamicBody);

	void processInputs();

	b2Vec2 getVectorComponent(float length, float angle);
	float degreesToRadians(float angle);
	float radiansToDegrees(float angle);

protected:

	b2Body* selected;
	bool applyingForce = false;
	b2Vec2 force;
	b2Vec2 globalPoint;
	float anguloPraLanca = 45.0;

};

//Aqui fazemos o registro do novo teste 
static int testIndex = RegisterTest("Examples", "Lista2", Lista2::Create);

b2Body* Lista2::CreateBox(b2Vec2 pos, b2Vec2 dim, float density, float friction, float restitution, b2BodyType type)
{
	b2Body* newBox;

	//Definição dos atributos gerais do corpo rígido
	b2BodyDef boxDef;

	boxDef.position = pos;
	boxDef.type = type;

	//Criação da forma do corpo*
	b2PolygonShape boxShape;

	boxShape.SetAsBox(dim.x, dim.y);

	//Definição da fixture
	b2FixtureDef boxFix;

	boxFix.shape = &boxShape;
	boxFix.density = density;
	boxFix.friction = friction;
	boxFix.restitution = restitution;

	//Criação do corpo rígido pelo mundo e da fixture pelo corpo rígido;
	newBox = m_world->CreateBody(&boxDef);
	newBox->CreateFixture(&boxFix);


	return newBox;
}

b2Body* Lista2::CreateCircle(b2Vec2 pos, float radius, float density, float friction, float restitution, b2BodyType type)
{
	b2Body* newCircle;

	//Definição dos atributos gerais do corpo rígido
	b2BodyDef circleDef;

	circleDef.position = pos;
	circleDef.type = type;


	//Criação da forma do corpo*
	b2CircleShape circleShape;

	circleShape.m_radius = radius;


	//Definição da fixture
	b2FixtureDef circleFix;

	circleFix.shape = &circleShape;
	circleFix.density = density;
	circleFix.friction = friction;
	circleFix.restitution = restitution;

	//Criação do corpo rígido pelo mundo e da fixture pelo corpo rígido;
	newCircle = m_world->CreateBody(&circleDef);
	newCircle->CreateFixture(&circleFix);

	return newCircle;
}

b2Body* Lista2::CreateLine(b2Vec2 pos, b2Vec2 inicio, b2Vec2 fim, float density, float friction, float restitution, b2BodyType type)
{
	b2Body* newLine;

	b2BodyDef lineDef;

	lineDef.position = pos;
	lineDef.type = type;

	b2EdgeShape lineShape;

	lineShape.SetTwoSided(inicio, fim);

	b2FixtureDef lineFix;

	lineFix.shape = &lineShape;
	lineFix.density = density;
	lineFix.friction = friction;
	lineFix.restitution = restitution;

	newLine = m_world->CreateBody(&lineDef);
	newLine->CreateFixture(&lineFix);

	return newLine;

}

void Lista2::processInputs()
{
	if (applyingForce)
	{
		force = getVectorComponent(10000, 0);
		//selected->ApplyForceToCenter(force, true);


		selected->ApplyForce(force, globalPoint, true);

		applyingForce = false;
	}
}

b2Vec2 Lista2::getVectorComponent(float length, float angle)
{
	b2Vec2 v;
	v.x = length * cos(degreesToRadians(angle));
	v.y = length * sin(degreesToRadians(angle));
	return v;
}

float Lista2::degreesToRadians(float angle)
{

	return angle * b2_pi / 180.0;
}

float Lista2::radiansToDegrees(float angle)
{
	return angle * 180.0 / b2_pi;
}
