using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Line;
using MGroup.MSolve.Discretization.Entities;

namespace RodTest
{
    public static class DiffusionRodCengel
    {
        public static Model CreateModel()
        {
            var model = new Model();
            model.SubdomainsDictionary.Add(0, new Subdomain(0));
            var nodes = new Node[]
            {
                new Node(id : 0, x : 0d,   y : 0d),
                new Node(id : 0, x : 1E-1, y : 0d),
                new Node(id : 0, x : 2E-1, y : 0d),
            };
            foreach (var node in nodes)
            {
                model.NodesDictionary.Add(node.ID, node);
            }
            
            var material = new ConvectionDiffusionProperties(capacityCoeff : 0d, diffusionCoeff : 1d, convectionCoeff : 0d, dependentSourceCoeff : 0d, independentSourceCoeff : 0d);

            var elements = new ConvectionDiffusionRod[]
            {
                new ConvectionDiffusionRod(new [] {nodes[0], nodes[1]}, 0.1, material),
                new ConvectionDiffusionRod(new [] {nodes[1], nodes[2]}, 0.1, material)
            };
            
            foreach (var element in elements)
            {
                model.ElementsDictionary.Add(element.ID, element);
                model.SubdomainsDictionary[0].Elements.Add(element);
            }

            model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
                new []
                {
                    new NodalUnknownVariable(nodes[0], ConvectionDiffusionDof.UnknownVariable, 120),
                    new NodalUnknownVariable(nodes[2], ConvectionDiffusionDof.UnknownVariable, 50)                    
                },
                new INodalConvectionDiffusionNeumannBoundaryCondition[] {} 
            ));
            return model;
        }

        public static Func<double, double> rodAnalyticalSolution = (x) => -350d * x + 120d;

        public static void CheckResults (double numericalSolution)
        {
            if ( numericalSolution - DiffusionRodCengel.rodAnalyticalSolution(1E-1) <= 1E-6)
            {
                Console.WriteLine("Mpravo sou! eisai o kalyteros!");
            }
            else
            {
                Console.WriteLine("WANK!");
            }
        }
    }
}
