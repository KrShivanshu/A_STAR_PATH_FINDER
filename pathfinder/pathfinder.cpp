#include "SFML/Graphics.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <list>

using namespace sf;
using namespace std;

class AStar
{
public:
	AStar()
	{
		OnUserCreate();
	}
	void setStart(int x,int y)
	{
		nodeStart = &nodes[y * nMapWidth + x];
	}
	void setEnd(int x, int y) {
		nodeEnd = &nodes[y * nMapWidth + x];
	}
	void setObstacle(int x, int y) {
		nodes[y * nMapWidth + x].bObstacle = true;
	}
	void unsetObstacle(int x, int y) {
		nodes[y * nMapWidth + x].bObstacle = false;
	}

	bool getVisitedState(int x, int y) {
		if (nodes[y * nMapWidth + x].bVisited == true)
			return 1;
		else
			return 0;
	}
	bool getParentState(int x,int y)
	{
		path();
		if (nodes[y * nMapWidth + x].isParent == true)
			return 1;
		else
			return 0;

	}
private:
	struct sNode
	{
		bool isParent = false;
		bool bObstacle = false;
		bool bVisited = false;
		float fGlobalGoal;
		float fLocalGoal;
		int x;
		int y;
		vector<sNode*> vecNeighbours;
		sNode* parent;

	};

	sNode *nodes = nullptr;
	int nMapWidth = 15;
	int nMapHeight = 20;

	sNode *nodeStart = nullptr;
	sNode *nodeEnd = nullptr;


protected:


	void OnUserCreate()
	{
		nodes = new sNode[nMapWidth * nMapHeight];
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
				nodes[y * nMapWidth + x].x = x;
				nodes[y * nMapWidth + x].y = y;
				nodes[y * nMapWidth + x].bObstacle = false;
				nodes[y * nMapWidth + x].parent = nullptr;
				nodes[y * nMapWidth + x].bVisited = false;

			}

		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
				if (y > 0)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x + 0)]);
				if (y < nMapHeight - 1)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x + 0)]);
				if (x > 0)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x - 1)]);
				if (x < nMapWidth - 1)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x + 1)]);
				/*
				if (y>0 && x>0)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x - 1)]);
				if (y<nMapHeight-1 && x>0)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x - 1)]);
				if (y>0 && x<nMapWidth-1)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x + 1)]);
				if (y<nMapHeight - 1 && x<nMapWidth-1)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x + 1)]);
				*/
			}


		nodeStart = &nodes[0];
		nodeEnd = &nodes[12];

	}
public:
	void Solve_AStar()
	{

		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
				nodes[y*nMapWidth + x].isParent = false;
				nodes[y*nMapWidth + x].bVisited = false;
				nodes[y*nMapWidth + x].fGlobalGoal = INFINITY;
				nodes[y*nMapWidth + x].fLocalGoal = INFINITY;
				nodes[y*nMapWidth + x].parent = nullptr;
			}

		auto distance = [](sNode* a, sNode* b)
		{
			return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
		};

		auto heuristic = [distance](sNode* a, sNode* b)
		{
			return distance(a, b);
		};

		sNode *nodeCurrent = nodeStart;
		nodeStart->fLocalGoal = 0.0f;
		nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

		list<sNode*> listNotTestedNodes;
		listNotTestedNodes.push_back(nodeStart);

		while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd)
		{

			listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs) { return lhs->fGlobalGoal < rhs->fGlobalGoal; });

			while (!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
				listNotTestedNodes.pop_front();

			if (listNotTestedNodes.empty())
				break;

			nodeCurrent = listNotTestedNodes.front();
			nodeCurrent->bVisited = true;

			for (auto nodeNeighbour : nodeCurrent->vecNeighbours)
			{
				if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)
					listNotTestedNodes.push_back(nodeNeighbour);

				float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);

				if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
				{
					nodeNeighbour->parent = nodeCurrent;
					nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;

					nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
				}
			}
		}
	}
	void path() {

		if (nodeEnd != nullptr)
		{
			sNode *p = nodeEnd;
			while (p->parent != nullptr)
			{

				nodes[p->y*nMapWidth + p->x].isParent = true;
				p = p->parent;
			}
		}
	}

};


int main(int argc, char ** argv) {
	AStar astar;
	const int boxOnX = 15, boxOnY = 20;
	RenderWindow window(sf::VideoMode(480.0f, 640.0f), "A-Star PathFinder", Style::Close | Style::Titlebar);
	RectangleShape box[boxOnX][boxOnY];
	const int boxSizeX = 32, boxSizeY = 32;
	for (int x = 0; x < boxOnX; x++)
		for (int y = 0; y < boxOnY; y++)
		{
			box[x][y].setSize(Vector2f(float(boxSizeX), float(boxSizeY)));
			box[x][y].setOutlineThickness(5.2f);
			box[x][y].setOutlineColor(Color::Black);
			box[x][y].setFillColor(Color::Blue);

		}
	box[0][0].setFillColor(Color::Green);
	box[12][0].setFillColor(Color::Red);
	while (window.isOpen()) {

		Event event;
		while (window.pollEvent(event)) {
			if (event.type == Event::EventType::Closed)
				window.close();

			if (Mouse::isButtonPressed(Mouse::Left))
			{
				Vector2i  mousePos = Mouse::getPosition(window);
				int x = floor(mousePos.x / boxSizeX);
				int y = floor(mousePos.y / boxSizeY);
				if (x <= 15 && x >= 0 && y <= 20 && y >= 0) {
					if (box[x][y].getFillColor() != Color::Green && box[x][y].getFillColor() != Color::Red) {
						box[x][y].setFillColor(Color::Magenta);
						astar.setObstacle(x, y);
					}
				}
				
			}
			if (Mouse::isButtonPressed(Mouse::Right))
			{
				Vector2i  mousePos = Mouse::getPosition(window);
				int x = floor(mousePos.x / boxSizeX);
				int y = floor(mousePos.y / boxSizeY);
				if (x <= 15 && x >= 0 && y <= 20 && y >= 0) 
				if (box[x][y].getFillColor() != Color::Green && box[x][y].getFillColor() != Color::Red) {
					box[x][y].setFillColor(Color::Blue);
					astar.unsetObstacle(x, y);
				}
			}
			if (Keyboard::isKeyPressed(Keyboard::LControl) || Keyboard::isKeyPressed(Keyboard::RControl)) {
				if (Mouse::isButtonPressed(Mouse::Left))
				{
					for (int x = 0; x < boxOnX; x++)
						for (int y = 0; y < boxOnY; y++)
						{
							if (box[x][y].getFillColor() == Color::Green) {
								box[x][y].setFillColor(Color::Blue);
								astar.unsetObstacle(x, y);
							}
						}
					Vector2i  mousePos = Mouse::getPosition(window);
					int x = floor(mousePos.x / boxSizeX);
					int y = floor(mousePos.y / boxSizeY);
					if (x <= 15 && x >= 0 && y <= 20 && y >= 0) {
						box[x][y].setFillColor(Color::Green);
						astar.setStart(x, y);
					}
				}
			}
			if (Keyboard::isKeyPressed(Keyboard::LShift) || Keyboard::isKeyPressed(Keyboard::RShift)) {
				if (Mouse::isButtonPressed(Mouse::Left))
				{

					for (int x = 0; x < boxOnX; x++)
						for (int y = 0; y < boxOnY; y++)
						{
							if (box[x][y].getFillColor() == Color::Red) {
								box[x][y].setFillColor(Color::Blue);
								astar.unsetObstacle(x, y);
							}
						}
					Vector2i  mousePos = Mouse::getPosition(window);
					int x = floor(mousePos.x / boxSizeX);
					int y = floor(mousePos.y / boxSizeY);
					if (x <= 15 && x >= 0 && y <= 20 && y >= 0) {
						box[x][y].setFillColor(Color::Red);
						astar.setEnd(x, y);
						//astar.setObstacle(x, y);
					}
				}
			}
			if (Keyboard::isKeyPressed(Keyboard::Return))
			{
				astar.Solve_AStar();
				for (int x = 0; x < boxOnX; x++)
					for (int y = 0; y < boxOnY; y++)
					{
						if (astar.getVisitedState(x, y) == 1 && !(box[x][y].getFillColor() == Color::Green) && !(box[x][y].getFillColor() == Color::Red))
							box[x][y].setFillColor(Color(255,255,204));

						if(astar.getVisitedState(x, y) == 0 && !(box[x][y].getFillColor() == Color::Green) && !(box[x][y].getFillColor() == Color::Red) && !(box[x][y].getFillColor() == Color::Magenta))
							box[x][y].setFillColor(Color::Blue);

						if (astar.getParentState(x, y) == 1 && !(box[x][y].getFillColor() == Color::Red))
						{
							box[x][y].setFillColor(Color::Yellow);
							//box[x][y].setOutlineThickness(1.2f);
							//box[x][y].setOutlineColor(Color::Yellow);
						}
					}
			}
		}
		window.clear(Color::Red);
		for (int x = 0; x < boxOnX; x++)
			for (int y = 0; y < boxOnY; y++)
			{
				box[x][y].setPosition(x * boxSizeX, y * boxSizeY);
				window.draw(box[x][y]);
			}
		window.display();
	}
	return 0;
}