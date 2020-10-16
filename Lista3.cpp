#include "test.h"
#include <time.h>


class Lista3 : public Test //você cria a sua classe derivada da classe base Test
{
public:

	Lista3() {

		CreateBox(b2Vec2(0.0, -3.8), b2Vec2(46.0, 1.0), 1.0, 0.5, 0.5, b2_staticBody); //bottom wall

		CreateBox(b2Vec2(0.0, 45.0), b2Vec2(46.0, 1.0), 1.0, 0.5, 0.5, b2_staticBody); // top wall

		CreateBox(b2Vec2(45.0, 20.5), b2Vec2(1.0, 25.5), 1.0, 0.5, 0.5, b2_staticBody); // right wall

		selected = CreateBox(b2Vec2(-45.0, 20.5), b2Vec2(1.0, 25.5), 1.0, 0.5, 0.5, b2_staticBody); //left wall

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
		g_debugDraw.DrawString(5, m_textLine, "Exercicios, utilize as teclas dos respectivos exercicios: '1', '2' + G,  3 - 'B' 'C' 'L' , '4' + 'B' , '5' , etc...");
		m_textLine += 15;

	}

	static Test* Create()  //a classe Test que instancia um objeto da sua nova classe
						   //o autor da Box2D usa um padrão de projeto chamado Factory
						   //para sua arquitetura de classes
	{

		return new Lista3;
	}

	//Para interagir com o teclado
	void Keyboard(int key) override //esse cabeçalho nao mudam
	{

		switch (key)
		{
			//Exercicio 1
		case GLFW_KEY_1:
		{
			b2Body* body1 = CreateBox(b2Vec2(0.0, 0.0), b2Vec2(5.0, 1.0), 10.0, 0.5, 0.5);
			b2Body* body2 = CreateCircle(b2Vec2(-2.0, -1.0), 1.0, 1.0, 0.5, 0.5);
			b2Body* body3 = CreateCircle(b2Vec2(2.0, -1.0), 1.0, 1.0, 0.5, 0.5);
			b2Body* body4 = CreateBox(b2Vec2(-10.0, -1.0), b2Vec2(1.0, 2.0), 1.0, 0.0, 0.0);
			b2Body* body5 = CreateBox(b2Vec2(3.1, 3.5), b2Vec2(0.5, 2), 1.0, 0.5, 0.5);
			b2Body* body6 = CreateCircle(b2Vec2(3.0, 10.0), 1.0, 1.0, 0.0, 0.5);

			b2RevoluteJointDef rJointDef;

			rJointDef.bodyA = body1;
			rJointDef.bodyB = body2;
			rJointDef.localAnchorA.Set(-2.3, -1.0);
			rJointDef.localAnchorB.Set(0.0, 0.0);

			b2RevoluteJointDef rJointDef2;
			rJointDef2.bodyA = body1;
			rJointDef2.bodyB = body3;
			rJointDef2.localAnchorA.Set(2.3, -1.0);
			rJointDef2.localAnchorB.Set(0.0, 0.0);

			b2PrismaticJointDef prismaticJointDef;
			prismaticJointDef.bodyA = body1;
			prismaticJointDef.bodyB = body4;

			prismaticJointDef.localAxisA.Normalize();

			prismaticJointDef.localAnchorA.Set(-6.0, -1.0);
			prismaticJointDef.localAnchorB.Set(1.0, -5.0); 

			b2RevoluteJointDef rJointDef3;
			b2Vec2 worldAnchorOnBody1 = body1->GetWorldPoint(b2Vec2(2, 1.0));
			rJointDef3.Initialize(body1, body5, worldAnchorOnBody1);
			rJointDef3.collideConnected = true;

			b2DistanceJointDef dJointDef;
			
			b2Vec2 worldAnchorOnBody2 = body5->GetWorldCenter();
			b2Vec2 worldAnchorOnBody3 = body6->GetWorldCenter();

			dJointDef.Initialize(body5, body6, worldAnchorOnBody2, worldAnchorOnBody3);
			dJointDef.collideConnected = true;	

			//Para limitar o angulo
			//rJointDef.lowerAngle = degreesToRadians(-45); // -45 degrees
			//rJointDef.upperAngle = degreesToRadians(45); // 45 degrees
			//rJointDef.enableLimit = true;

			//Para criar um motor
			rJointDef.maxMotorTorque = 1000.0f; //N*m
			rJointDef.motorSpeed = 100.0f; //radianos por segundo 
			rJointDef.enableMotor = true;

			rJointDef2.maxMotorTorque = 1000.0f;
			rJointDef2.motorSpeed = 100.0f;
			rJointDef2.enableMotor = true;

			b2RevoluteJoint* rjoint = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef);
			b2RevoluteJoint* rJoint2 = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef2);
			b2PrismaticJoint* m_joint = (b2PrismaticJoint*)m_world->CreateJoint(&prismaticJointDef);
			b2RevoluteJoint* rJoint3 = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef3);
			b2DistanceJoint* dJoint = (b2DistanceJoint*)m_world->CreateJoint(&dJointDef);

			break;
		}

		//Exercicio 2
		case GLFW_KEY_2:
		{
			b2Body* body1 = CreateBox(b2Vec2(-20.0, 2.2), b2Vec2(1.0, 5.0), 10.0, 0.5, 0.5);
			b2Body* body2 = CreateBox(b2Vec2(-16.0, 5.5), b2Vec2(3.0, 1.0), 1.0, 0.0, 0.0);
			b2Body* body3 = CreateBox(b2Vec2(-10.0, 5.5), b2Vec2(3.0, 1.0), 1.0, 0.5, 0.5);
			b2Body* body4 = CreateBox(b2Vec2(-4.0, 5.5), b2Vec2(3.0, 1.0), 1.0, 0.0, 0.0);
			b2Body* body5 = CreateBox(b2Vec2(2.0, 5.5), b2Vec2(3.0, 1.0), 1.0, 0.5, 0.5);
			b2Body* body6 = CreateBox(b2Vec2(8.0, 5.5), b2Vec2(3.0, 1.0), 1.0, 0.0, 0.0);
			b2Body* body7 = CreateBox(b2Vec2(14.0, 5.5), b2Vec2(3.0, 1.0), 1.0, 0.5, 0.5);
			b2Body* body8 = CreateBox(b2Vec2(18.0, 2.2), b2Vec2(1.0, 5.0), 1.0, 0.5, 0.5);


			b2RevoluteJointDef rJointDef;
			b2Vec2 worldAnchorOnBody1 = body1->GetWorldPoint(b2Vec2(0.0, 0.0));
			rJointDef.Initialize(body1, body2, worldAnchorOnBody1);
			rJointDef.collideConnected = true;

			b2RevoluteJointDef rJointDef2;
			b2Vec2 worldAnchorOnBody2 = body2->GetWorldPoint(b2Vec2(0.0, 0.0));
			rJointDef2.Initialize(body2, body3, worldAnchorOnBody2);
			rJointDef2.collideConnected = true;

			b2RevoluteJointDef rJointDef3;
			b2Vec2 worldAnchorOnBody3 = body3->GetWorldPoint(b2Vec2(0.0, 0.0));
			rJointDef3.Initialize(body3, body4, worldAnchorOnBody3);
			rJointDef3.collideConnected = true;

			b2RevoluteJointDef rJointDef4;
			b2Vec2 worldAnchorOnBody4 = body4->GetWorldPoint(b2Vec2(0.0, 0.0));
			rJointDef4.Initialize(body4, body5, worldAnchorOnBody4);
			rJointDef4.collideConnected = true;

			b2RevoluteJointDef rJointDef5;
			b2Vec2 worldAnchorOnBody5 = body5->GetWorldPoint(b2Vec2(0.0, 0.0));
			rJointDef5.Initialize(body5, body6, worldAnchorOnBody5);
			rJointDef5.collideConnected = true;

			b2RevoluteJointDef rJointDef6;
			b2Vec2 worldAnchorOnBody6 = body6->GetWorldPoint(b2Vec2(0.0, 0.0));
			rJointDef6.Initialize(body6, body7, worldAnchorOnBody6);
			rJointDef6.collideConnected = true;

			b2RevoluteJointDef rJointDef7;
			b2Vec2 worldAnchorOnBody7 = body7->GetWorldPoint(b2Vec2(0.0, 0.0));
			rJointDef7.Initialize(body7, body8, worldAnchorOnBody7);
			rJointDef7.collideConnected = true;

			b2RevoluteJoint* rjoint = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef);
			b2RevoluteJoint* rjoint2 = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef2);
			b2RevoluteJoint* rjoint3 = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef3);
			b2RevoluteJoint* rjoint4 = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef4);
			b2RevoluteJoint* rjoint5 = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef5);
			b2RevoluteJoint* rjoint6 = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef6);
			b2RevoluteJoint* rjoint7 = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef7);

			break;
		}
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
static int testIndex = RegisterTest("Examples", "Lista3", Lista3::Create);

b2Body* Lista3::CreateBox(b2Vec2 pos, b2Vec2 dim, float density, float friction, float restitution, b2BodyType type)
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

b2Body* Lista3::CreateCircle(b2Vec2 pos, float radius, float density, float friction, float restitution, b2BodyType type)
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

b2Body* Lista3::CreateLine(b2Vec2 pos, b2Vec2 inicio, b2Vec2 fim, float density, float friction, float restitution, b2BodyType type)
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

void Lista3::processInputs()
{
	if (applyingForce)
	{
		force = getVectorComponent(10000, 0);
		//selected->ApplyForceToCenter(force, true);


		selected->ApplyForce(force, globalPoint, true);

		applyingForce = false;
	}
}

b2Vec2 Lista3::getVectorComponent(float length, float angle)
{
	b2Vec2 v;
	v.x = length * cos(degreesToRadians(angle));
	v.y = length * sin(degreesToRadians(angle));
	return v;
}

float Lista3::degreesToRadians(float angle)
{

	return angle * b2_pi / 180.0;
}

float Lista3::radiansToDegrees(float angle)
{
	return angle * 180.0 / b2_pi;
}
