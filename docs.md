# Документация для Physnova 1.0

## Здесь прописаны только те методы, атрибуты, функции, которые вы можете использовать в своих целях. За исключением лишь статических классов *Resolver* и *Collisions*, так как их методы здесь не прописаны, но вы можете их использовать для создания более продвинутого поведения

*Класс World*
1) Главный класс в игре, отвечает за работу практически всех остальных классов
2) Функции:
    * CameraLookAt({точка}, {плавность}) - камера следит за точкой с определённой плавностью
    * ZoomCamera({значение [float]}), MoveCamera({значение [pygame.Vector2]}) - камера приближается/отдаляется на выбранное значение или увеличивает позицию на значение
    * CreateCircleBody({материал}, {статический ли}, {радиус}, {заморозить ли повороты}) - создаёт тело формы окружности, помещает его в список и возвращает новое тело
    * CreateBoxBody({материал}, {статический ли}, {ширина}, {высота}, {заморозить ли повороты}) - создаёт тело прямоугольной формы
    * RemoveBody({тело}) - удаляет тело из списка
    * RemoveBody({индекс}) - удаляет тело из списка по индексу
    * GetRigidbody({индекс}) - возвращает тело в списке под данным индексом
    * Update({экран}, {Δt}, {кол-во итераций}) - обновляет мир. Δt - отношение 1 к кол-ву fps (например, 1/60). Кол-во итераций определяет точность разрешения столкновений, но уменьшает fps. То есть чем больше итераций, тем лучше разрешаются столкновения и меньше fps

*Статический Класс Resolver* - отвечает за разрешение столкновений

*Статический Класс Collisions* - отвечает за обнаружение столкновений

*Класс Rigidbody* 
1) Класс тела. Вместо привычной записи "body = Rigidbody({параметры})" лучше использовать метод CreateCircleBody/CreateBoxBody класса *World*
2) Функции:
    * Move/SetPosition ({значение, [pygame.Vector2]}) - двигает объект/устанавливает позицию
    * ApplyForce({значение, [pygame.Vector2]}) - прикладывает силу
    * Rotate/SetAngle ({значение}, [float]) - вращает объект/устанавливает угол (в радианах, поэтому лучше использовать функцию модуля math - math.radians({угол}))
    * ApplyAngularForce({значение}, [float]) - прикладывает силу вращения
    * GetTransformedVertices() - возвращает вершины прямоугольника
    * GetSurface() - возвращает поверхность объекта
    * SetSurface({поверхность}) - устанавливает поверхность объекту. Если тип формы объекта "окружность", то обрезает поверхность по углам.
    * CreateSurface({цвет заливки}, {цвет обводки}, {ширина обводки}) - создаёт новую поверхность с выбранными параметрами. Если ширина обводки равна нулю, то обводки не будет.
3) Атрибуты (все атрибуты изменять нельзя, только просматривать значение, иначе это может привести к сбоям в работе движка):
    * Shape - форма [str]
    * Surface - поверхность [pygame.Surface]
    * Radius - радиус (если форма "прямоугольник", то равен 0) [float]
    * Width - ширина (если форма "окружность", то равна 0) [float]
    * Height - высота (если форма "окружность", то равна 0) [float]
    * Mass - масса [float]
    * InvMass - инвертированная масса [float]
    * Area - площадь [float]
    * Inertia - инерция поворота [float]
    * InvInertia - инвертированная инерция поворота [float]
    * Material - материал [physnova.Material]
    * IsStatic - статический ли объект [bool]
    * Position - позиция [pygame.Vector2]
    * LinearVelocity - линейная скорость [pygame.Vector2]
    * Angle - угол поворота [float]
    * AngularVelocity - скорость поворота [float]
    * Force - сила [pygame.Vector2]

*Класс Material*
1) Класс материала.
2) При создании:
    * density - плотность в г/см³ [float] (0.5-23)
    * restitution - коэффициент жёсткости [float] (0-1)
    * staticFriction - коэффициент трения покоя [float] (0-1)
    * dynamicFriction - коэффициент трения скольжения/качения [float] (0-1)